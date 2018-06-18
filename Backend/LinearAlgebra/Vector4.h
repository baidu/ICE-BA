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
#ifndef _VECTOR_4_H_
#define _VECTOR_4_H_

#include "Utility.h"

namespace LA {

class AlignedVector4f {

 public:

  inline const xp128f &v0123() const { return m_data; }  inline xp128f &v0123() { return m_data; }

  inline const float& v0() const { return m_data[0]; }    inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }    inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }    inline float& v2() { return m_data[2]; }
  inline const float& v3() const { return m_data[3]; }    inline float& v3() { return m_data[3]; }


  inline const xp128f& xyzw() const { return v0123(); }  inline xp128f& xyzw() { return v0123(); }
  inline const float& x() const { return v0(); }  inline float& x() { return v0(); }
  inline const float& y() const { return v1(); }  inline float& y() { return v1(); }
  inline const float& z() const { return v2(); }  inline float& z() { return v2(); }
  inline const float& w() const { return v3(); }  inline float& w() { return v3(); }

  inline operator const float* () const { return (const float *) this; }
  inline operator float* () { return (float *) this; }

  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline bool operator == (const AlignedVector4f &v) const {
    return v0() == v.v0() && v1() == v.v1() && v2() == v.v2() && v3() == v.v3();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline AlignedVector4f operator - (const AlignedVector4f &b) const {
    AlignedVector4f amb;
    amb.v0123() = v0123() - b.v0123();
    return amb;
  }

  inline void Set(const float *v) { memcpy(this, v, 16); }
  inline void Set(const double *v) {
    v0123().vset_all_lane(float(v[0]), float(v[1]), float(v[2]), float(v[3]));
  }
  inline void Get(float *v) const { memcpy(v, this, 16); }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedVector4f)); }
  inline void MakeMinus() {
    v0123().vmake_minus();
  }
  inline AlignedVector4f GetMinus() const {
    AlignedVector4f v;
    xp128f zero; zero.vdup_all_lane(0.f);
    v.v0123() = zero - v0123();
    return v;
  }

  inline bool Valid() const { return v0() != FLT_MAX; }
  inline bool Invalid() const { return v0() == FLT_MAX; }
  inline void Invalidate() { v0() = FLT_MAX; }

  inline void Scale(const float s) {
    v0123() *= s;
  }
  inline void Scale(const xp128f &s) {
    v0123() *= s;
  }
  inline float SquaredLength() const {
    return (v0123() * v0123()).vsum_all();
  }
  inline float Dot(const AlignedVector4f &v) const {
    return (v0123() * v.v0123()).vsum_all();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e\n", v0(), v1(), v2(), v3());
    } else {
      UT::Print("%f %f %f %f\n", v0(), v1(), v2(), v3());
    }
  }
  inline void Save(FILE *fp, const bool e = false) const {
    if (e) {
      fprintf(fp, "%e %e %e %e\n", v0(), v1(), v2(), v3());
    } else {
      fprintf(fp, "%f %f %f %f\n", v0(), v1(), v2(), v3());
    }
  }
  inline void Load(FILE *fp) {
#ifdef CFG_DEBUG
    const int N = fscanf(fp, "%f %f %f %f", &v0(), &v1(), &v2(), &v3());
    UT_ASSERT(N == 4);
#else
    fscanf(fp, "%f %f %f %f", &v0(), &v1(), &v2(), &v3());
#endif
  }

  inline void Random(const float vMax) { UT::Random(&v0(), 4, -vMax, vMax); }
  static inline AlignedVector4f GetRandom(const float vMax) {
    AlignedVector4f v;
    v.Random(vMax);
    return v;
  }

  inline bool AssertEqual(const AlignedVector4f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&v0(), &v.v0(), 4, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const AlignedVector4f e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&v0(), 4, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  inline void SetInfinite() {
    v0123().vdup_all_lane(FLT_MAX);
  }
  inline void AssertInfinite() const {
    UT_ASSERT(v0() == FLT_MAX);
    UT_ASSERT(v1() == FLT_MAX);
    UT_ASSERT(v2() == FLT_MAX);
    UT_ASSERT(v3() == FLT_MAX);
  }
  inline void AssertFinite() const {
    UT_ASSERT(v0() != FLT_MAX);
    UT_ASSERT(v1() != FLT_MAX);
    UT_ASSERT(v2() != FLT_MAX);
    UT_ASSERT(v3() != FLT_MAX);
  }

 protected:
  xp128f m_data;
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector4f : public Eigen::Vector4f {
 public:
  inline EigenVector4f() : Eigen::Vector4f() {}
  inline EigenVector4f(const Eigen::Vector4f &e_v) : Eigen::Vector4f(e_v) {}
  inline EigenVector4f(const LA::AlignedVector4f &v) : Eigen::Vector4f(v.v0(), v.v1(), v.v2(),
                                                                         v.v3()) {}
  inline EigenVector4f(const float v0, const float v1, const float v2,
                       const float v3) : Eigen::Vector4f(v0, v1, v2, v3) {}
  inline void operator = (const Eigen::Vector4f &e_v) { *((Eigen::Vector4f *) this) = e_v; }
  inline LA::AlignedVector4f GetAlignedVector4f() const {
    LA::AlignedVector4f v;
    const Eigen::Vector4f &e_v = *this;
    v.v0() = e_v(0);
    v.v1() = e_v(1);
    v.v2() = e_v(2);
    v.v3() = e_v(3);
    return v;
  }
  inline float SquaredLength() const { return GetAlignedVector4f().SquaredLength(); }
  inline void Print(const bool e = false) const { GetAlignedVector4f().Print(e); }
  static inline EigenVector4f GetRandom(const float vMax) { return EigenVector4f(LA::AlignedVector4f::GetRandom(vMax)); }
  inline bool AssertEqual(const LA::AlignedVector4f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector4f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVector4f &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVector4f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
