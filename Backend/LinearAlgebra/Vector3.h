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
#ifndef LINEARALGEBRA_VECTOR3_H_
#define LINEARALGEBRA_VECTOR3_H_

#include "Vector2.h"
#include "Utility.h"
#include <string>
#include <algorithm>

namespace LA {

class AlignedVector3f {
 public:
  inline AlignedVector3f() {}
  inline AlignedVector3f(const float *v) { Set(v); }
  inline AlignedVector3f(const float v0, const float v1, const float v2) {
    v012r().vset_all_lane(v0, v1, v2, 0.0f);
  }
  inline AlignedVector3f(const Vector2f &x, const float z) {
    xyzr().vset_all_lane(x.x() * z, x.y() * z, z, 0.0f);
  }
  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline AlignedVector3f(const xp128f &v) : m_data(v) {}  // NOLINT

  inline const xp128f &v012r() const { return m_data; }   inline xp128f &v012r() { return m_data; }
  inline const xp128f &xyzr () const { return m_data; }   inline xp128f &xyzr () { return m_data; }

  inline const float& v0() const { return m_data[0]; }  inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }  inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }  inline float& v2() { return m_data[2]; }
  inline const float& r () const { return m_data[3]; }  inline float& r () { return m_data[3]; }
  inline const float& x () const { return m_data[0]; }  inline float& x () { return m_data[0]; }
  inline const float& y () const { return m_data[1]; }  inline float& y () { return m_data[1]; }
  inline const float& z () const { return m_data[2]; }  inline float& z () { return m_data[2]; }

  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline void operator = (const xp128f &v) {
    v012r() = v;
  }

  inline bool operator == (const AlignedVector3f &v) const {
    // TODO(yanghongtian) : rooms for optimizations
    return v0() == v.v0() && v1() == v.v1() && v2() == v.v2();
  }
  inline void operator += (const AlignedVector3f &v) {
    v012r() += v.v012r();
  }
  inline void operator -= (const AlignedVector3f &v) {
    v012r() -= v.v012r();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline void operator *= (const xp128f &s) { v012r() *= s; }
  inline void operator *= (const AlignedVector3f &s) { v012r() *= s.v012r(); }
  inline void operator /= (const float d) { Scale(1.0f / d); }
  inline AlignedVector3f operator + (const AlignedVector3f &b) const {
    AlignedVector3f _apb;
    apb(*this, b, _apb);
    return _apb;
  }
  inline AlignedVector3f operator + (const float b) const {
    AlignedVector3f apb;
    apb.v012r() = v012r() + b;
    return apb;
  }
  inline AlignedVector3f operator - (const AlignedVector3f &b) const {
    AlignedVector3f _amb;
    amb(*this, b, _amb);
    return _amb;
  }

  inline AlignedVector3f operator - (const float b) const {
    AlignedVector3f amb;
    amb.v012r() = v012r() - b;
    return amb;
  }
  inline AlignedVector3f operator * (const float s) const {
    AlignedVector3f v;
    GetScaled(s, v);
    return v;
  }
  inline AlignedVector3f operator * (const xp128f &s) const {
    AlignedVector3f v;
    GetScaled(s, v);
    return v;
  }

  inline void Set(const float *v) {
    memcpy(&m_data[0], v, 12);
  }
  inline void Set(const double *v) {
    v012r().vset_all_lane(static_cast<float>(v[0]),
                          static_cast<float>(v[1]),
                          static_cast<float>(v[2]),
                          0.0f);
  }
  inline void Set(const float v0, const float v1, const float v2) {
    v012r().vset_all_lane(v0, v1, v2, 0.0f);
  }
  inline void Set(const Vector2f &x, const float z) {
    xyzr().vset_all_lane(x.x() * z, x.y() * z, z, 0.0f);
  }
  inline void Get(float *v) const { memcpy(v, this, 12); }
  inline void Get(double *v) const {
    v[0] = static_cast<double>(v0());
    v[1] = static_cast<double>(v1());
    v[2] = static_cast<double>(v2());
  }
  inline void GetMinus(AlignedVector3f &v) const {
    const xp128f zero = xp128f::get(0.0f);
    v.v012r() = zero - v012r();
  }
  inline AlignedVector3f GetMinus() const { AlignedVector3f v; GetMinus(v); return v; }

  inline void MakeZero() { memset(this, 0, 12); }
  inline void MakeMinus() { v012r().vmake_minus();}

  inline bool Valid() const { return v0() != FLT_MAX; }
  inline bool Invalid() const { return v0() == FLT_MAX; }
  inline void Invalidate() { v0() = FLT_MAX; }

  inline void Project(Vector2f &x) const {
    x.y()  = 1.f / z();
    x.x() = this->x() * x.y();
    x.y() = this->y() * x.y();
  }
  inline const Vector2f GetProjected() const { Vector2f x; Project(x); return x; }

  inline void Normalize() {
    v012r() *= (1.0f / sqrtf(SquaredLength()));
  }
  inline AlignedVector3f GetNormalized() const {
    AlignedVector3f v;
    v.v012r() = v012r() * (1.0f / sqrtf(SquaredLength()));
    return v;
  }

  inline void Scale(const float s) {
    v012r() *= s;
  }
  inline void GetScaled(const float s, AlignedVector3f &v) const {
    v.v012r() = v012r() * s;
  }
  inline void GetScaled(const xp128f &s, AlignedVector3f &v) const {
    v.v012r() = v012r() * s;
  }
  inline void GetScaled(const LA::AlignedVector3f &s, AlignedVector3f &v) const {
    v.v012r() = v012r() * s.v012r();
  }
  inline AlignedVector3f GetScaled(const float s) const {
    AlignedVector3f v;
    GetScaled(s, v);
    return v;
  }
  inline void GetSquareRoot(AlignedVector3f &v) const {
    v.v012r().vset_sqrt_from_vec(v012r());
  }
  inline AlignedVector3f GetSquareRoot() const {
    AlignedVector3f v; GetSquareRoot(v); return v;
  }
  inline void MakeSquareRoot() {
    v012r().vset_sqrt_from_vec(v012r());
  }

  inline float Sum() const { return v012r().vsum_012(); }
  inline float SquaredLength() const {
    return (v012r() * v012r()).vsum_012();
  }
  inline float Dot(const AlignedVector3f &v) const {
    return (v012r() * v.v012r()).vsum_012();
  }
  inline float Dot(const float *v) const {
    return (v012r() * *(reinterpret_cast<const xp128f *>(v))).vsum_012();
  }
  inline AlignedVector3f Cross(const AlignedVector3f &v) const {
    AlignedVector3f c;
    c.x() = y() * v.z() - z() * v.y();
    c.y() = z() * v.x() - x() * v.z();
    c.z() = x() * v.y() - y() * v.x();
    return c;
  }

  inline void Interpolate(const float w1, const AlignedVector3f &v1, const AlignedVector3f &v2) {
    v012r() = v1.v012r() * w1 + v2.v012r() * (1.f - w1);
  }
  inline void Interpolate(const AlignedVector3f &v1, const AlignedVector3f &v2,
                          const float t1, const float t2, const float t) {
    const float w1 = (t2 - t) / (t2 - t1);
    Interpolate(w1, v1, v2);
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e\n", v0(), v1(), v2());
    } else {
      UT::Print("%f %f %f\n", v0(), v1(), v2());
    }
  }
  inline void Print(const std::string str, const bool e, const bool l, const bool n) const {
    UT::Print("%s", str.c_str());
    if (e) {
      UT::Print("%e %e %e", v0(), v1(), v2());
    } else {
      UT::Print("%f %f %f", v0(), v1(), v2());
    }
    if (l) {
      if (e) {
        UT::Print(" = %e", sqrtf(SquaredLength()));
      } else {
        UT::Print(" = %f", sqrtf(SquaredLength()));
      }
    }
    if (n) {
      UT::Print("\n");
    }
  }
  inline void Save(FILE *fp, const bool e = false) const {
    if (e) {
      fprintf(fp, "%e %e %e\n", v0(), v1(), v2());
    } else {
      fprintf(fp, "%f %f %f\n", v0(), v1(), v2());
    }
  }

  inline void Load(FILE *fp) {
#ifdef CFG_DEBUG
    const int N = fscanf(fp, "%f %f %f", &v0(), &v1(), &v2());
    UT_ASSERT(N == 3);
#else
    fscanf(fp, "%f %f %f", &v0(), &v1(), &v2());
#endif
  }

  inline bool AssertEqual(const AlignedVector3f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&v0(), &v.v0(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const AlignedVector3f e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&v0(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  inline void SetInfinite() {
    v012r().vdup_all_lane(FLT_MAX);
  }
  inline void AssertInfinite() const {
    UT_ASSERT(v0() == FLT_MAX);
    UT_ASSERT(v1() == FLT_MAX);
    UT_ASSERT(v2() == FLT_MAX);
    UT_ASSERT(r() == FLT_MAX);
  }
  inline void AssertFinite() const {
    UT_ASSERT(v0() != FLT_MAX);
    UT_ASSERT(v1() != FLT_MAX);
    UT_ASSERT(v2() != FLT_MAX);
    UT_ASSERT(r() != FLT_MAX);
  }

  inline void Random(const float vMax) { Random(-vMax, vMax); }
  inline void Random(const float vMin, const float vMax) { UT::Random(&v0(), 3, vMin, vMax); }
  static inline AlignedVector3f GetRandom(const float vMax) {
    AlignedVector3f v;
    v.Random(vMax);
    return v;
  }
  static inline AlignedVector3f GetRandom(const float vMin, const float vMax) {
    AlignedVector3f v;
    v.Random(vMin, vMax);
    return v;
  }

  static inline void apb(const AlignedVector3f &a,
                         const AlignedVector3f &b, AlignedVector3f &apb) {
    apb.v012r() = a.v012r() + b.v012r();
  }
  static inline void amb(const AlignedVector3f &a,
                         const AlignedVector3f &b, AlignedVector3f &amb) {
    amb.v012r() = a.v012r() - b.v012r();
  }

 protected:
  xp128f m_data;
};

template<typename TYPE> class Vector3 {
 public:
  //inline Vector3<TYPE>() {}
  //inline Vector3<TYPE>(const TYPE *v) { Set(v); }
  //inline Vector3<TYPE>(const TYPE v0, const TYPE v1, const TYPE v2) {
  //  this->v0() = v0;
  //  this->v1() = v1;
  //  this->v2() = v2;
  //}
  //inline Vector3<TYPE>(const Vector2<TYPE> &v0, const TYPE v1) {
  //  this->v0() = v0.v0();
  //  this->v1() = v0.v1();
  //  this->v2() = v1;
  //}
   static inline Vector3<TYPE> Get(const TYPE v0, const TYPE v1, const TYPE v2) {
     Vector3<TYPE> v;
     v.Set(v0, v1, v2);
     return v;
   }

  inline const TYPE& v0() const { return m_data[0]; }   inline TYPE& v0() { return m_data[0]; }
  inline const TYPE& v1() const { return m_data[1]; }   inline TYPE& v1() { return m_data[1]; }
  inline const TYPE& v2() const { return m_data[2]; }   inline TYPE& v2() { return m_data[2]; }
  inline const TYPE& x () const { return m_data[0]; }   inline TYPE& x () { return m_data[0]; }
  inline const TYPE& y () const { return m_data[1]; }   inline TYPE& y () { return m_data[1]; }
  inline const TYPE& z () const { return m_data[2]; }   inline TYPE& z () { return m_data[2]; }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }
  inline void operator = (const TYPE *v) { Set(v); }
  inline void operator += (const Vector3<TYPE> &v) {
    v0() = v.v0() + v0();
    v1() = v.v1() + v1();
    v2() = v.v2() + v2();
  }
  inline void operator -= (const Vector3<TYPE> &v) {
    v0() = -v.v0() + v0();
    v1() = -v.v1() + v1();
    v2() = -v.v2() + v2();
  }
  inline void operator += (const AlignedVector3f &v) {
    v0() = v.v0() + v0();
    v1() = v.v1() + v1();
    v2() = v.v2() + v2();
  }
  inline void operator *= (const TYPE s) { Scale(s); }
  inline Vector3<TYPE> operator + (const Vector3<TYPE> &v) const {
    return Vector3<TYPE>::Get(v0() + v.v0(), v1() + v.v1(), v2() + v.v2());
  }
  inline Vector3<TYPE> operator - (const Vector3<TYPE> &v) const {
    return Vector3<TYPE>::Get(v0() - v.v0(), v1() - v.v1(), v2() - v.v2());
  }
  inline Vector3<TYPE> operator * (const TYPE s) const {
    return Vector3<TYPE>::Get(v0() * s, v1() * s, v2() * s);
  }

  inline void Set(const TYPE v0, const TYPE v1, const TYPE v2) {
    this->v0() = v0;
    this->v1() = v1;
    this->v2() = v2;
  }
  inline void Set(const float *v);
  inline void Set(const double *v);
  inline void Get(TYPE *v) const { memcpy(v, this, sizeof(Vector3<TYPE>)); }
  inline AlignedVector3f GetAlignedVector3f() const { AlignedVector3f v; Get(v); return v; }
  inline void GetMinus(Vector3<TYPE> &v) const {
    v.v0() = -v0(); v.v1() = -v1(); v.v2() = -v2();
  }
  inline void MakeMinus() { v0() = -v0(); v1() = -v1(); v2() = -v2(); }

  inline void MakeZero() { memset(this, 0, sizeof(Vector3<TYPE>)); }

  inline void Scale(const TYPE s) { v0() *= s; v1() *= s; v2() *= s; }
  inline TYPE Dot(const Vector3<TYPE> &v) const {
    return v0() * v.v0() + v1() * v.v1() + v2() * v.v2();
  }
  inline void Normalize() { Scale(1 / sqrtf(SquaredLength())); }
  inline TYPE SquaredLength() const { return v0() * v0() + v1() * v1() + v2() * v2(); }
  inline TYPE Sum() const { return v0() + v1() + v2(); }
  inline TYPE Maximal() const { return std::max(std::max(v0(), v1()), v2()); }
  inline TYPE Minimal() const { return std::min(std::min(v0(), v1()), v2()); }

  inline void Invalidate() { v0() = UT::Invalid<TYPE>(); }
  inline bool Valid() const { return v0() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return v0() == UT::Invalid<TYPE>(); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e\n", v0(), v1(), v2());
    } else {
      UT::Print("%f %f %f\n", v0(), v1(), v2());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    UT::Print("%s", str.c_str());
    Print(e);
  }

  inline bool AssertEqual(const Vector3<TYPE> &v,
                          const int verbose = 1, const std::string str = "",
                          const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    if (UT::VectorAssertEqual<TYPE>(&v0(), &v.v0(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const Vector3<TYPE> e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }

  inline void Random(const TYPE vMin, const TYPE vMax) { UT::Random(&v0(), 3, vMin, vMax); }
  static inline Vector3<TYPE> GetRandom(const TYPE vMin, const TYPE vMax) {
    Vector3<TYPE> v;
    v.Random(vMin, vMax);
    return v;
  }

 protected:
  TYPE m_data[3];
};

typedef Vector3<int>  Vector3i;
typedef Vector3<float>  Vector3f;
typedef Vector3<double> Vector3d;
typedef Vector3<ubyte>  Vector3ub;

template<> inline void Vector3f::Set(const float *v) { memcpy(this, v, sizeof(Vector3f)); }
template<> inline void Vector3d::Set(const double *v) { memcpy(this, v, sizeof(Vector3d)); }
template<> inline void Vector3f::Set(const double *v) {
  v0() = static_cast<float>(v[0]);
  v1() = static_cast<float>(v[1]);
  v2() = static_cast<float>(v[2]);
}
template<> inline void Vector3d::Set(const float *v) {
  v0() = static_cast<double>(v[0]);
  v1() = static_cast<double>(v[1]);
  v2() = static_cast<double>(v[2]);
}
}  // namespace LA

#ifdef CFG_DEBUG_EIGEN
class EigenVector3f : public Eigen::Vector3f {
 public:
  inline EigenVector3f() : Eigen::Vector3f() {}
  inline EigenVector3f(const Eigen::Vector3f &e_v) : Eigen::Vector3f(e_v) {}
  inline EigenVector3f(const float *v) : Eigen::Vector3f(v[0], v[1], v[2]) {}
  inline EigenVector3f(const LA::AlignedVector3f &v)
    : Eigen::Vector3f(v.v0(), v.v1(), v.v2()) { }
  inline EigenVector3f(const LA::Vector3f &v) : Eigen::Vector3f(v.v0(), v.v1(), v.v2()) {}
  inline EigenVector3f(const float v0, const float v1, const float v2) : Eigen::Vector3f(v0, v1,
                                                                                         v2) {}
  inline EigenVector3f(const EigenVector2f &e_v0, const float v1)
    : Eigen::Vector3f(e_v0(0), e_v0(1), v1) { }
  inline void operator = (const Eigen::Vector3f &e_v) { *((Eigen::Vector3f *) this) = e_v; }
  inline LA::AlignedVector3f GetAlignedVector3f() const {
    LA::AlignedVector3f v;
    const Eigen::Vector3f &e_v = *this;
    v.v0() = e_v(0);
    v.v1() = e_v(1);
    v.v2() = e_v(2);
    return v;
  }
  inline LA::Vector3f GetVector3f() const {
    LA::Vector3f v;
    const Eigen::Vector3f &e_v = *this;
    v.v0() = e_v(0);
    v.v1() = e_v(1);
    v.v2() = e_v(2);
    return v;
  }
  inline EigenVector2f Project() const {
    EigenVector2f e_x;
    e_x.y() = 1.0f / z();
    e_x.x() = x() * e_x.y();
    e_x.y() = y() * e_x.y();
    return e_x;
  }
  inline float SquaredLength() const { return GetAlignedVector3f().SquaredLength(); }
  inline void Print(const bool e = false) const { GetAlignedVector3f().Print(e); }
  static inline EigenVector3f GetRandom(const float vMax) {
    return EigenVector3f(LA::AlignedVector3f::GetRandom(vMax));
  }
  inline bool AssertEqual(const LA::AlignedVector3f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector3f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::Vector3f &v, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetVector3f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVector3f &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVector3f(), verbose, str, epsAbs, epsRel);
  }
  static inline EigenVector3f Zero() { return EigenVector3f(Eigen::Vector3f::Zero()); }
};

#endif
#endif  // LINEARALGEBRA_VECTOR3_H_
