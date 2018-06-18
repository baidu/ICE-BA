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
#ifndef _VECTOR_7_H_
#define _VECTOR_7_H_

#include "Vector6.h"
#include "Utility.h"

namespace LA {

class AlignedVector7f {
 public:
  inline const xp128f &v0123() const { return m_data4[0]; } inline xp128f &v0123() { return m_data4[0]; }
  inline const xp128f &v456x() const { return m_data4[1]; } inline xp128f &v456x() { return m_data4[1]; }
  inline const float& v0() const { return m_data[0]; }      inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }      inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }      inline float& v2() { return m_data[2]; }
  inline const float& v3() const { return m_data[3]; }      inline float& v3() { return m_data[3]; }
  inline const float& v4() const { return m_data[4]; }      inline float& v4() { return m_data[4]; }
  inline const float& v5() const { return m_data[5]; }      inline float& v5() { return m_data[5]; }
  inline const float& v6() const { return m_data[6]; }      inline float& v6() { return m_data[6]; }


  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }

  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline void operator += (const AlignedVector7f &v) {
    v0123() += v.v0123();
    v456x() += v.v456x();
  }
  inline void operator -= (const AlignedVector7f &v) {
    v0123() -= v.v0123();
    v456x() -= v.v456x();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline void operator *= (const xp128f &s) { Scale(s); }
  inline AlignedVector7f operator - (const AlignedVector7f &b) const {
    AlignedVector7f amb;
    amb.v0123() = v0123() - b.v0123();
    amb.v456x() = v456x() - b.v456x();
    return amb;
  }
  inline AlignedVector7f operator * (const xp128f &s) const {
    AlignedVector7f v;
    GetScaled(s, v);
    return v;
  }

  inline void Set(const AlignedVector6f &v) { memcpy(this, &v, 24); }
  inline void Set(const float *v) { memcpy(this, v, sizeof(AlignedVector7f)); }
  inline void Get(float *v) const { memcpy(v, this, sizeof(AlignedVector7f)); }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedVector7f)); }
  inline void MakeMinus() {
    v0123().vmake_minus();
    v456x().vmake_minus();
  }

  inline void Scale(const float s) {
    xp128f _s; _s.vdup_all_lane(s);
    Scale(_s);
  }
  inline void Scale(const xp128f &s) {
    v0123() *= s;
    v456x() *= s;
  }
  inline void GetScaled(const xp128f &s, AlignedVector7f &v) const {
    v.v0123() = s * v0123();
    v.v456x() = s * v456x();
  }

  inline float SquaredLength() const {
  #ifdef CFG_DEBUG
    UT_ASSERT(m_data4[1][3] == 0.0f);
  #endif
    // Make sure m_data4[1][3] is always 0
    return (v0123() * v0123() + v456x() * v456x()).vsum_all();
  }
  inline float Dot(const AlignedVector7f &v) const {
  #ifdef CFG_DEBUG
    UT_ASSERT(m_data4[1][3] == 0.0f || v.m_data4[1][3] == 0.0f);
  #endif
    // Make sure m_data4[1][3] is always 0
    return (v0123() * v.v0123() + v456x() * v.v456x()).vsum_all();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5(), v6());
    } else {
      UT::Print("%f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6());
    }
  }

  static inline void apb(const AlignedVector7f &a, const AlignedVector7f &b, AlignedVector7f &apb) {
    apb.v0123() = a.v0123() + b.v0123();
    apb.v456x() = a.v456x() + b.v456x();
  }

  inline void Random(const float vMax) { UT::Random(&v0(), 7, -vMax, vMax); }
  static inline AlignedVector7f GetRandom(const float vMax) { AlignedVector7f v; v.Random(vMax); return v; }

  inline bool AssertEqual(const AlignedVector7f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&v0(), &v.v0(), 7, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const AlignedVector7f e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&v0(), 7, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

 protected:
  union {
    float m_data[7];
    xp128f m_data4[2];
  };
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector7f : public Eigen::Matrix<float, 7, 1> {
 public:
  inline EigenVector7f() : Eigen::Matrix<float, 7, 1>() {}
  inline EigenVector7f(const Eigen::Matrix<float, 7, 1> &e_v) : Eigen::Matrix<float, 7, 1>(e_v) {}
  inline EigenVector7f(const LA::AlignedVector7f &v) : Eigen::Matrix<float, 7, 1>() {
    const float* _v = v;
    Eigen::Matrix<float, 7, 1> &e_v = *this;
    for (int i = 0; i < 7; ++i)
      e_v(i, 0) = _v[i];
  }
  inline void operator = (const Eigen::Matrix<float, 7, 1> &e_v) {
    *((Eigen::Matrix<float, 7, 1> *) this) = e_v;
  }
  inline LA::AlignedVector7f GetAlignedVector7f() const {
    LA::AlignedVector7f v;
    float* _v = v;
    const Eigen::Matrix<float, 7, 1> &e_v = *this;
    for (int i = 0; i < 7; ++i)
      _v[i] = e_v(i, 0);
    return v;
  }
  inline float SquaredLength() const { return GetAlignedVector7f().SquaredLength(); }
  inline void Print(const bool e = false) const { GetAlignedVector7f().Print(e); }
  static inline EigenVector7f GetRandom(const float vMax) {
    return EigenVector7f(LA::AlignedVector7f::GetRandom(vMax));
  }
  inline bool AssertEqual(const LA::AlignedVector7f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector7f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVector7f &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVector7f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
