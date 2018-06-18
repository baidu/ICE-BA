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
#ifndef _VECTOR_8_H_
#define _VECTOR_8_H_

#include "Utility.h"

namespace LA {

class AlignedVector8f {

 public:

  inline AlignedVector8f() {}
  inline AlignedVector8f(const float *v) { Set(v); }

  inline const xp128f &v0123() const { return m_data4[0]; } inline xp128f &v0123() { return m_data4[0]; }
  inline const xp128f &v4567() const { return m_data4[1]; } inline xp128f &v4567() { return m_data4[1]; }
  inline const float& v0() const { return m_data[0]; }      inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }      inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }      inline float& v2() { return m_data[2]; }
  inline const float& v3() const { return m_data[3]; }      inline float& v3() { return m_data[3]; }
  inline const float& v4() const { return m_data[4]; }      inline float& v4() { return m_data[4]; }
  inline const float& v5() const { return m_data[5]; }      inline float& v5() { return m_data[5]; }
  inline const float& v6() const { return m_data[6]; }      inline float& v6() { return m_data[6]; }
  inline const float& v7() const { return m_data[7]; }      inline float& v7() { return m_data[7]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline void operator += (const AlignedVector8f &v) {
    v0123() += v.v0123();
    v4567() += v.v4567();
  }
  inline void operator -= (const AlignedVector8f &v) {
    v0123() -= v.v0123();
    v4567() -= v.v4567();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline void operator *= (const xp128f &s) { Scale(s); }
  inline AlignedVector8f operator - (const AlignedVector8f &v) const {
    AlignedVector8f amb;
    AlignedVector8f::amb(*this, v, amb);
    return amb;
  }

  inline AlignedVector8f operator + (const AlignedVector8f &v) const {
    AlignedVector8f apb;
    AlignedVector8f::apb(*this, v, apb);
    return apb;
  }
  inline AlignedVector8f operator * (const xp128f &s) const {
    AlignedVector8f v;
    GetScaled(s, v);
    return v;
  }

  inline void Set(const float *v) { memcpy(this, v, sizeof(AlignedVector8f)); }
  inline void Get(float *v) const { memcpy(v, this, sizeof(AlignedVector8f)); }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedVector8f)); }
  inline void MakeMinus() {
    v0123().vmake_minus();
    v4567().vmake_minus();
  }

  inline void Scale(const float s) {
    xp128f _s; _s.vdup_all_lane(s);
    Scale(_s);
  }
  inline void Scale(const xp128f &s) {
    v0123() *= s;
    v4567() *= s;
  }
  inline void GetScaled(const xp128f &s, AlignedVector8f &v) const {
    v.v0123() = v0123() * s;
    v.v4567() = v4567() * s;
  }
  inline float SquaredLength() const {
    return (v0123() * v0123() + v4567() * v4567()).vsum_all();
  }
  inline float Dot(const AlignedVector8f &v) const {
    return (v0123() * v.v0123() + v4567() *  v.v4567()).vsum_all();
  }

  inline void Print(const bool e = false) const {
    if (e)
      UT::Print("%e %e %e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7());
    else
      UT::Print("%f %f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7());
  }

  static inline void apb(const AlignedVector8f &a, const AlignedVector8f &b, AlignedVector8f &apb) {
    apb.v0123() = a.v0123() + b.v0123();
    apb.v4567() = a.v4567() + b.v4567();
  }
  static inline void amb(const AlignedVector8f &a, const AlignedVector8f &b, AlignedVector8f &amb) {
    amb.v0123() = a.v0123() - b.v0123();
    amb.v4567() = a.v4567() - b.v4567();
  }

  inline void Random(const float vMax) { UT::Random(&v0(), 8, -vMax, vMax); }
  static inline AlignedVector8f GetRandom(const float vMax) { AlignedVector8f v; v.Random(vMax); return v; }

  inline bool AssertEqual(const LA::AlignedVector8f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&v0(), &v.v0(), 8, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const AlignedVector8f e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&v0(), 8, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

 protected:
  union {
    float m_data[8];
    xp128f m_data4[2];
  };
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector8f : public Eigen::Matrix<float, 8, 1> {
 public:
  inline EigenVector8f() : Eigen::Matrix<float, 8, 1>() {}
  inline EigenVector8f(const Eigen::Matrix<float, 8, 1> &e_v) : Eigen::Matrix<float, 8, 1>(e_v) {}
  inline EigenVector8f(const LA::AlignedVector8f &v) : Eigen::Matrix<float, 8, 1>() {
    const float* _v = v;
    Eigen::Matrix<float, 8, 1> &e_v = *this;
    for (int i = 0; i < 8; ++i)
      e_v(i, 0) = _v[i];
  }
  inline void operator = (const Eigen::Matrix<float, 8, 1> &e_v) { *((Eigen::Matrix<float, 8, 1> *) this) = e_v; }
  inline LA::AlignedVector8f GetAlignedVector8f() const {
    LA::AlignedVector8f v;
    float* _v = v;
    const Eigen::Matrix<float, 8, 1> &e_v = *this;
    for (int i = 0; i < 8; ++i)
      _v[i] = e_v(i, 0);
    return v;
  }
  inline float SquaredLength() const { return GetAlignedVector8f().SquaredLength(); }
  inline void Print(const bool e = false) const { GetAlignedVector8f().Print(e); }
  static inline EigenVector8f GetRandom(const float vMax) {
    return EigenVector8f(LA::AlignedVector8f::GetRandom(vMax));
  }
  inline bool AssertEqual(const LA::AlignedVector8f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector8f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVector8f &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVector8f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
