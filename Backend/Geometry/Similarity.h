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
#ifndef _SIMILARITY_H_
#define _SIMILARITY_H_

#include "Rotation.h"

class Similarity3D : public Rotation3D {

 public:

  inline const xp128f& sr00_sr01_sr02_tx() const { return r_00_01_02_x(); }
  inline       xp128f& sr00_sr01_sr02_tx()       { return r_00_01_02_x(); }
  inline const xp128f& sr10_sr11_sr12_ty() const { return r_10_11_12_x(); }
  inline       xp128f& sr10_sr11_sr12_ty()       { return r_10_11_12_x(); }
  inline const xp128f& sr20_sr21_sr22_tz() const { return r_20_21_22_x(); }
  inline       xp128f& sr20_sr21_sr22_tz() { return r_20_21_22_x(); }

  inline const float& sr00() const { return r00(); }    inline float& sr00() { return r00(); }
  inline const float& sr01() const { return r01(); }    inline float& sr01() { return r01(); }
  inline const float& sr02() const { return r02(); }    inline float& sr02() { return r02(); }
  inline const float& tx  () const { return r0 (); }    inline float& tx  () { return r0 (); }
  inline const float& sr10() const { return r10(); }    inline float& sr10() { return r10(); }
  inline const float& sr11() const { return r11(); }    inline float& sr11() { return r11(); }
  inline const float& sr12() const { return r12(); }    inline float& sr12() { return r12(); }
  inline const float& ty  () const { return r1 (); }    inline float& ty  () { return r1 (); }
  inline const float& sr20() const { return r20(); }    inline float& sr20() { return r20(); }
  inline const float& sr21() const { return r21(); }    inline float& sr21() { return r21(); }
  inline const float& sr22() const { return r22(); }    inline float& sr22() { return r22(); }
  inline const float& tz  () const { return r2 (); }    inline float& tz  () { return r2 (); }

  inline Point3D operator * (const Point3D &X) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    Point3D SX;
    Apply(X, SX);
    return SX;
  }

  inline void Set(const float s12, const Quaternion &q1, const Quaternion &q2, const Point3D &p1,
                  const Point3D &p2) {
    Quaternion q12;
    Quaternion::AIB(q2, q1, q12);
    m_s = _mm_set1_ps(s12);
    m_R.SetQuaternion(q12);
    m_t = p2 - Rotation3D::GetApplied(p1);
    Update();
  }
  inline void Update() {
    m_R.GetScaled(m_s, *this);
    tx() = m_t.v0();
    ty() = m_t.v1();
    tz() = m_t.v2();
  }

  inline void Apply(const Point3D &X, LA::AlignedVector3f &SX) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    SX.x() = SIMD::Sum(_mm_mul_ps(sr00_sr01_sr02_tx(), X.xyzw()));
    SX.y() = SIMD::Sum(_mm_mul_ps(sr10_sr11_sr12_ty(), X.xyzw()));
    SX.z() = SIMD::Sum(_mm_mul_ps(sr20_sr21_sr22_tz(), X.xyzw()));
  }
  inline void ApplyRotation(const Point3D &X, LA::AlignedVector3f &RX) const { m_R.Apply(X, RX); }
  inline void ApplyRotationScale(const Point3D &X, LA::AlignedVector3f &sRX) const { Rotation3D::Apply(X, sRX); }
  inline LA::AlignedVector3f GetApplied(const Point3D &X) const { LA::AlignedVector3f SX; Apply(X, SX); return SX; }

  inline void Print(const bool e = false) const {
    UT::PrintSeparator();
    if (e) {
      UT::Print(" T = %e %e %e %e\n", sr00(), sr01(), sr02(), tx());
      UT::Print("     %e %e %e %e\n", sr10(), sr11(), sr12(), ty());
      UT::Print("     %e %e %e %e\n", sr20(), sr21(), sr22(), tz());
      UT::Print(" s = %e\n", m_s[0]);
      UT::Print(" R = %e %e %e\n", r00(), r01(), r02());
      UT::Print("     %e %e %e\n", r10(), r11(), r12());
      UT::Print("     %e %e %e\n", r20(), r21(), r22());
      UT::Print(" t = %e %e %e\n", tx(), ty(), tz());
    } else {
      UT::Print(" T = %f %f %f %f\n", sr00(), sr01(), sr02(), tx());
      UT::Print("     %f %f %f %f\n", sr10(), sr11(), sr12(), ty());
      UT::Print("     %f %f %f %f\n", sr20(), sr21(), sr22(), tz());
      UT::Print(" s = %e\n", m_s[0]);
      UT::Print(" R = %f %f %f\n", r00(), r01(), r02());
      UT::Print("     %f %f %f\n", r10(), r11(), r12());
      UT::Print("     %f %f %f\n", r20(), r21(), r22());
      UT::Print(" t = %f %f %f\n", tx(), ty(), tz());
    }
  }

 public:

  xp128f m_s;
  Rotation3D m_R;
  LA::AlignedVector3f m_t;
};

#ifdef CFG_DEBUG_EIGEN
class EigenSimilarity3D {
 public:
  inline EigenSimilarity3D() {}
#ifdef _MSC_VER
  inline EigenSimilarity3D(const Similarity3D &S) : m_s(S.m_s[0]), m_R(S.m_R), m_t(S.m_t) {}
#else
  inline EigenSimilarity3D(const Similarity3D &S) : m_s(S.m_s[0]), m_R(S.m_R), m_t(S.m_t) {}
#endif  // _MSC_VER
  inline EigenSimilarity3D(const float e_s, const EigenRotation3D &e_R,
                           const EigenVector3f &e_t) : m_s(e_s), m_R(e_R), m_t(e_t) {}
  inline EigenVector3f operator * (const EigenVector3f &e_X) const {
    return EigenVector3f(m_s * m_R * e_X + m_t);
  }
  inline Similarity3D GetSimilarity() const {
    Similarity3D S;
    S.m_s = _mm_set1_ps(m_s);
    S.m_R = m_R.GetAlignedMatrix3x3f();
    S.m_t = m_t.GetAlignedVector3f();
    S.Update();
    return S;
  }
 public:
  float m_s;
  EigenRotation3D m_R;
  EigenVector3f m_t;
};
#endif

#endif
