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
#ifndef _ROTATION_H_
#define _ROTATION_H_

#include "Matrix3x3.h"
#include "Matrix6x6.h"
#include "Vector4.h"
#include "Point.h"

class AxisAngle : public LA::AlignedVector4f {

 public:

  inline AxisAngle() {}
  inline AxisAngle(const LA::AlignedVector3f &k, const float th) { xyzw() = k.xyzr(); w() = th; }

  inline void MakeIdentity() {
    xyzw().vset_all_lane(1.0f, 0.0f, 0.0f, 0.0f);
  }

  inline void FromVectors(const LA::AlignedVector3f &v1, const LA::AlignedVector3f &v2) {
    SIMD::Cross012(v1.xyzr(), v2.xyzr(), xyzw());
    w() = UT_ACOSF(v1.Dot(v2));
    Normalize();
  }

  inline void SetRodrigues(const LA::AlignedVector3f &w, const float eps) {
    const float th = sqrtf(w.SquaredLength());
    if (th < eps) {
      MakeIdentity();
    } else {
      this->xyzw() = w.xyzr() * (1.0f / th);
      this->w() = th;
    }
  }
  inline void GetRodrigues(LA::AlignedVector3f &w) const {
    const float th = this->w() < UT_PI ? this->w() : this->w() - UT_2PI;
    w.xyzr() = this->xyzw() * th;
  }

  inline void Normalize() {
    const float th = w();
    xyzw() *= 1.f / sqrtf((xyzw() * xyzw()).vsum_012());
    w() = th;
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("(%e %e %e) %e\n", x(), y(), z(), w() * UT_FACTOR_RAD_TO_DEG);
    } else {
      UT::Print("(%f %f %f) %f\n", x(), y(), z(), w() * UT_FACTOR_RAD_TO_DEG);
    }
  }

  inline void Random(const float thMax) {
    LA::AlignedVector4f::Random(1.0f);
    w() *= thMax;
    Normalize();
  }
  static inline AxisAngle GetRandom(const float thMax) { AxisAngle kth; kth.Random(thMax); return kth; }

  inline bool AssertEqual(const AxisAngle &kth, const int verbose = 1,
                          const std::string str = "") const {
    if (fabs(w()) < FLT_EPSILON && fabs(kth.w()) < FLT_EPSILON) {
      return true;
    }
    AxisAngle kth1 = *this, kth2 = kth;
    if (kth1.w() > UT_PI) {
      kth1.w() = kth1.w() - UT_2PI;
    }
    if (kth2.w() > UT_PI) {
      kth2.w() = kth2.w() - UT_2PI;
    }
    if ((kth1.w() > 0.0f && kth2.w() < 0.0f) ||
        (kth1.w() < 0.0f && kth2.w() > 0.0f)) {
      kth2.MakeMinus();
    }
    //return kth1.LA::AlignedVector4f::AssertEqual(kth2, verbose);
    //if (UT::AssertEqual(SIMD::Dot012(kth1.xyzw(), kth2.xyzw()), 1.0f) && UT::AssertEqual(kth1.w(), kth2.w()))
    const float d = kth1.xyzw().vdot012(kth2.xyzw());
    //if (UT::AssertEqual(UT_DOT_TO_ANGLE(d), 0.0f) && UT::AssertEqual(kth1.w(), kth2.w()))
    const float eps = 0.1f;
    //const float eps = 0.01f;
    if (UT::AssertZero(UT_DOT_TO_ANGLE(d), verbose, str + ".dot", eps * UT_FACTOR_DEG_TO_RAD) &&
        UT::AssertEqual(kth1.w(), kth2.w(), verbose, str + ".m_w")) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      kth1.Print(verbose > 1);
      kth2.Print(verbose > 1);
    }
    return false;
  }
};

class Quaternion : public LA::AlignedVector4f {

 public:

  inline Quaternion() {}
  inline Quaternion(const LA::AlignedVector3f &w, const float eps) {
    SetRodrigues(w, eps);
  }
  inline Quaternion(const LA::AlignedVector4f &q) { xyzw() = q.xyzw(); }

  inline void Get(float *v) const { memcpy(v, this, 16); }

  inline Quaternion operator * (const Quaternion &q) const {
    Quaternion _q;
    AB(*this, q, _q);
    return _q;
  }
  inline Quaternion operator / (const Quaternion &q) const {
    Quaternion _q;
    ABI(*this, q, _q);
    return _q;
  }

  inline void SetRodrigues(const LA::AlignedVector3f &w, const float eps) {
    const float th2 = w.SquaredLength(), th = sqrtf(th2);
    if (th < eps) {
      const float s = 1.0f / sqrtf(th2 + 4.0f);
      this->xyzw() = w.xyzr() * s;
      this->w() = s + s;
    } else {
      const float thh = th * 0.5f;
      this->xyzw() = w.xyzr() * (UT_SINF(thh) / th);
      this->w() = UT_COSF(thh);
    }
  }
  inline void GetRodrigues(LA::AlignedVector3f &w, const float eps) const {
    const float thh = UT_DOT_TO_ANGLE(this->w()), th = thh + thh;
    if (th < eps) {
      w.MakeZero();
    } else {
      w.xyzr() = xyzw() * (th / UT_SINF(thh));
    }
  }
  inline void SetAxisAngle(const AxisAngle &kth) {
    const float thh = kth.w() * 0.5f;
    xyzw() = kth.xyzw() * UT_SINF(thh);
    w() = UT_COSF(thh);
  }
  inline void GetAxisAngle(AxisAngle &kth) const {
    kth.xyzw() = xyzw();
    kth.w() = UT_ACOSF(w()) * 2.0f;
    kth.Normalize();
  }
  inline void GetGravity(LA::Vector3f &g) const {
    const xp128f t1 = xyzw() * x();
    const xp128f t2 = xyzw() * y();

    g.x() = t1[2] - t2[3];
    g.y() = t2[2] + t1[3];
    g.z() = t1[0] + t2[1];

    g.x() = -(g.x() + g.x());
    g.y() = -(g.y() + g.y());
    g.z() = g.z() + g.z() - 1.0f;
  }
  inline LA::Vector3f GetGravity() const {
    LA::Vector3f g;
    GetGravity(g);
    return g;
  }


  inline void MakeIdentity() { xyzw().vset_all_lane(0.0f, 0.0f, 0.0f, 1.0f); }
  inline void MakeZero() { MakeIdentity(); }

  inline void Normalize() {
    const float s = 1.0f / sqrtf(SquaredLength());
    Scale(w() > 0.0f ? s : -s);
  }
  inline void Inverse() { w() = -w(); }
  inline Quaternion GetInverse() const { Quaternion q = *this; q.Inverse(); return q; }

  inline void Slerp(const float w1, const Quaternion &q1, const Quaternion &q2) {
    x() = q1.Dot(q2);
    if (x() > 0.0f) {
      if (x() > 1.0f) {
        x() = 0.0f;
      //} else if (x() < -1.0f)
      //  x() = UT_PI;
      } else {
        x() = UT_ACOSF(x());
      }
      if (fabs(x()) < FLT_EPSILON) {
        *this = q1;
        return;
      }
      y() = 1 / UT_SINF(x());
      const float s1 = UT_SINF(w1 * x()) * y();
      const float s2 = UT_SINF((1 - w1) * x()) * y();
      xyzw() = q1.xyzw() * s1 + q2.xyzw() * s2;
      Normalize();
    } else {
      x() = -x();
      if (x() > 1.0f) {
        x() = 0.0f;
      //} else if (x() < -1.0f) {
      //  x() = UT_PI;
      } else {
        x() = UT_ACOSF(x());
      }
      if (fabs(x()) < FLT_EPSILON) {
        *this = q1;
        return;
      }
      y() = 1 / UT_SINF(x());
      const float s1 = UT_SINF(w1 * x()) * y();
      const float s2 = UT_SINF((1 - w1) * x()) * y();
      xyzw() = q1.xyzw() * s1 - q2.xyzw() * s2;
    }
  }
  inline void Slerp(const Quaternion &q1, const Quaternion &q2, const float t1, const float t2,
                    const float t) {
    const float w1 = (t2 - t) / (t2 - t1);
    Slerp(w1, q1, q2);
  }
  static inline float GetAngle(const Quaternion &q1, const Quaternion &q2) {
    const float d = q1.Dot(q2);
    return UT_DOT_TO_ANGLE(d) * 2.0f;
  }

  inline void Random(const float thMax) {
    AxisAngle kth;
    kth.Random(thMax);
    SetAxisAngle(kth);
  }
  static inline Quaternion GetRandom(const float thMax) { Quaternion q; q.Random(thMax); return q; }

  inline bool AssertEqual(const Quaternion &q, const int verbose = 1, const std::string str = "",
                          const float eps = 0.001745329252f) const {
    Quaternion q1 = *this, q2 = q;
    if ((q1.w() > 0.0f && q2.w() < 0.0f) ||
        (q1.w() < 0.0f && q2.w() > 0.0f)) {
      q2.MakeMinus();
    }
    //return q1.LA::AlignedVector4f::AssertEqual(q2, verbose, str, epsAbs, epsRel);
    //const float eps = 0.1f * UT_FACTOR_DEG_TO_RAD;
    if (UT::AssertZero(GetAngle(q1, q2), verbose, str, eps)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      q.Print(verbose > 1);
    }
    return false;
  }

  static inline void AB(const Quaternion &A, const Quaternion &B, Quaternion &AB) {
    xp128f t;
    t = A.xyzw() * B.x();
    AB.x() =  t[3];
    AB.y() = -t[2];
    AB.z() =  t[1];
    AB.w() = -t[0];
    t = A.xyzw() * B.y();
    AB.x() =  t[2] + AB.x();
    AB.y() =  t[3] + AB.y();
    AB.z() = -t[0] + AB.z();
    AB.w() = -t[1] + AB.w();
    t = A.xyzw() * B.z();
    AB.x() = -t[1] + AB.x();
    AB.y() =  t[0] + AB.y();
    AB.z() =  t[3] + AB.z();
    AB.w() = -t[2] + AB.w();
    t = A.xyzw() * B.w();
    AB.x() =  t[0] + AB.x();
    AB.y() =  t[1] + AB.y();
    AB.z() =  t[2] + AB.z();
    AB.w() =  t[3] + AB.w();
    AB.Normalize();
  }

  static inline void ABI(const Quaternion &A, const Quaternion &B, Quaternion &AB) {
    xp128f t;
    t = A.xyzw() * B.x();
    AB.x() =  t[3];
    AB.y() = -t[2];
    AB.z() =  t[1];
    AB.w() = -t[0];
    t = A.xyzw() * B.y();
    AB.x() =  t[2] + AB.x();
    AB.y() =  t[3] + AB.y();
    AB.z() = -t[0] + AB.z();
    AB.w() = -t[1] + AB.w();
    t = A.xyzw() * B.z();
    AB.x() = -t[1] + AB.x();
    AB.y() =  t[0] + AB.y();
    AB.z() =  t[3] + AB.z();
    AB.w() = -t[2] + AB.w();
    t = A.xyzw() * (-B.w());
    AB.x() =  t[0] + AB.x();
    AB.y() =  t[1] + AB.y();
    AB.z() =  t[2] + AB.z();
    AB.w() =  t[3] + AB.w();
    AB.Normalize();
  }

  static inline void AIB(const Quaternion &A, const Quaternion &B, Quaternion &AIB) {
    xp128f t;
    t = A.xyzw() * B.x();
    AIB.x() = -t[3];
    AIB.y() = -t[2];
    AIB.z() =  t[1];
    AIB.w() = -t[0];
    t = A.xyzw() * B.y();
    AIB.x() =  t[2] + AIB.x();
    AIB.y() = -t[3] + AIB.y();
    AIB.z() = -t[0] + AIB.z();
    AIB.w() = -t[1] + AIB.w();
    t = A.xyzw() * B.z();
    AIB.x() = -t[1] + AIB.x();
    AIB.y() =  t[0] + AIB.y();
    AIB.z() = -t[3] + AIB.z();
    AIB.w() = -t[2] + AIB.w();
    t = A.xyzw() * B.w();
    AIB.x() =  t[0] + AIB.x();
    AIB.y() =  t[1] + AIB.y();
    AIB.z() =  t[2] + AIB.z();
    AIB.w() = -t[3] + AIB.w();
    AIB.Normalize();
  }
};

class SkewSymmetricMatrix : public LA::AlignedVector3f {

 public:

  inline SkewSymmetricMatrix() {}
  inline SkewSymmetricMatrix(const LA::AlignedVector3f &w) : LA::AlignedVector3f(w) {}

  inline LA::AlignedMatrix3x3f operator * (const LA::AlignedMatrix3x3f &B) const { LA::AlignedMatrix3x3f _AB; AB(*this, B, _AB); return _AB; }
  inline LA::AlignedVector3f operator * (const LA::AlignedVector3f &b) {
    LA::AlignedVector3f Ab;
    SkewSymmetricMatrix::Ab(*this, b, Ab);
    return Ab;
  }
  inline LA::AlignedVector3f operator * (const xp128f &s) {
    AlignedVector3f v;
    GetScaled(s, v);
    return v;
  }

  inline float m00() const { return 0.0f; }
  inline float m01() const { return -z(); }
  inline float m02() const { return y(); }
  inline float m10() const { return z(); }
  inline float m11() const { return 0.0f; }
  inline float m12() const { return -x(); }
  inline float m20() const { return -y(); }
  inline float m21() const { return x(); }
  inline float m22() const { return 0.0f; }

  inline void Get(LA::AlignedMatrix3x3f &M) const {
    M.m00() = m00();  M.m01() = m01();  M.m02() = m02();
    M.m10() = m10();  M.m11() = m11();  M.m12() = m12();
    M.m20() = m20();  M.m21() = m21();  M.m22() = m22();
  }
  inline LA::AlignedMatrix3x3f GetAlignedMatrix3x3f() const {
    LA::AlignedMatrix3x3f M;
    Get(M);
    return M;
  }
  inline void GetSquared(LA::SymmetricMatrix3x3f &M) const {
    M.SetRow0(v012r() * v0());
    M.m11() = v1() * v1();
    M.m12() = v1() * v2();
    M.m22() = v2() * v2();
    const float l2 = M.m00() + M.m11() + M.m22();
    M.m00() -= l2;
    M.m11() -= l2;
    M.m22() -= l2;
  }

  inline void Transpose() { MakeMinus(); }
  inline void GetTranspose(SkewSymmetricMatrix &AT) const { GetMinus(AT); }
  static inline void GetTranspose(const LA::AlignedVector3f &v, LA::AlignedMatrix3x3f &M) {
    M.m00() = 0.0f;   M.m01() = v.z();  M.m02() = -v.y();
    M.m10() = -v.z(); M.m11() = 0.0f;   M.m12() = v.x();
    M.m20() = v.y();  M.m21() = -v.x(); M.m22() = 0.0f;
  }

  static inline void Ab(const LA::AlignedVector3f &a, const LA::AlignedVector3f &b,
                        LA::AlignedVector3f &Ab) {
    Ab.x() = a.y() * b.z() - a.z() * b.y();
    Ab.y() = a.z() * b.x() - a.x() * b.z();
    Ab.z() = a.x() * b.y() - a.y() * b.x();
  }
  static inline void AddAbTo(const LA::AlignedVector3f &a, const LA::AlignedVector3f &b,
                             LA::AlignedVector3f &Ab) {
    Ab.x() = a.y() * b.z() - a.z() * b.y() + Ab.x();
    Ab.y() = a.z() * b.x() - a.x() * b.z() + Ab.y();
    Ab.z() = a.x() * b.y() - a.y() * b.x() + Ab.z();
  }
  static inline void AddATbTo(const LA::AlignedVector3f &a, const LA::AlignedVector3f &b,
                              LA::AlignedVector3f &ATb) {
    ATb.x() = a.z() * b.y() - a.y() * b.z() + ATb.x();
    ATb.y() = a.x() * b.z() - a.z() * b.x() + ATb.y();
    ATb.z() = a.y() * b.x() - a.x() * b.y() + ATb.z();
  }
  static inline void AB(const LA::AlignedVector3f &a, const LA::AlignedMatrix3x3f &B,
                        LA::AlignedMatrix3x3f &AB) {
    const float ax = a.x(), ay = a.y(), az = a.z();
    AB.m_00_01_02_r0() = B.m_20_21_22_r2() * ay - B.m_10_11_12_r1() * az;
    AB.m_10_11_12_r1() = B.m_00_01_02_r0() * az - B.m_20_21_22_r2() * ax;
    AB.m_20_21_22_r2() = B.m_10_11_12_r1() * ax - B.m_00_01_02_r0() * ay;
  }
  static inline void ATB(const LA::AlignedVector3f &a, const LA::AlignedMatrix3x3f &B,
                         LA::AlignedMatrix3x3f &AB) {
    const float ax = a.x(), ay = a.y(), az = a.z();
    AB.m_00_01_02_r0() = B.m_10_11_12_r1() * az - B.m_20_21_22_r2() * ay;
    AB.m_10_11_12_r1() = B.m_20_21_22_r2() * ax - B.m_00_01_02_r0() * az;
    AB.m_20_21_22_r2() = B.m_00_01_02_r0() * ay - B.m_10_11_12_r1() * ax;
  }
  static inline void AddABTo(const LA::AlignedVector3f &a, const LA::AlignedMatrix3x3f &B,
                             LA::AlignedMatrix3x3f &AB) {    
    const float ax = a.x(), ay = a.y(), az = a.z();
    AB.m_00_01_02_r0() += B.m_20_21_22_r2() * ay - B.m_10_11_12_r1() * az;
    AB.m_10_11_12_r1() += B.m_00_01_02_r0() * az - B.m_20_21_22_r2() * ax;
    AB.m_20_21_22_r2() += B.m_10_11_12_r1() * ax - B.m_00_01_02_r0() * ay;
  }
  static inline void AB(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                        LA::AlignedMatrix3x3f &AB) {
    AB.m00() = A.m01() * b.z() - A.m02() * b.y();
    AB.m01() = A.m02() * b.x() - A.m00() * b.z();
    AB.m02() = A.m00() * b.y() - A.m01() * b.x();
    AB.m10() = A.m11() * b.z() - A.m12() * b.y();
    AB.m11() = A.m12() * b.x() - A.m10() * b.z();
    AB.m12() = A.m10() * b.y() - A.m11() * b.x();
    AB.m20() = A.m21() * b.z() - A.m22() * b.y();
    AB.m21() = A.m22() * b.x() - A.m20() * b.z();
    AB.m22() = A.m20() * b.y() - A.m21() * b.x();
  }
  static inline void ATB(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                         LA::AlignedMatrix2x3f &AB) {
    AB.m00() = A.m10() * b.z() - A.m20() * b.y();
    AB.m01() = A.m20() * b.x() - A.m00() * b.z();
    AB.m02() = A.m00() * b.y() - A.m10() * b.x();
    AB.m10() = A.m11() * b.z() - A.m21() * b.y();
    AB.m11() = A.m21() * b.x() - A.m01() * b.z();
    AB.m12() = A.m01() * b.y() - A.m11() * b.x();
  }
  static inline void ATBT(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                         LA::AlignedMatrix2x3f &AB) {
    AB.m00() = A.m20() * b.y() - A.m10() * b.z();
    AB.m01() = A.m00() * b.z() - A.m20() * b.x();
    AB.m02() = A.m10() * b.x() - A.m00() * b.y();
    AB.m10() = A.m21() * b.y() - A.m11() * b.z();
    AB.m11() = A.m01() * b.z() - A.m21() * b.x();
    AB.m12() = A.m11() * b.x() - A.m01() * b.y();
  }
  static inline void AddABTo(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                             LA::AlignedMatrix3x3f &AB) {
    AB.m00() = A.m01() * b.z() - A.m02() * b.y() + AB.m00();
    AB.m01() = A.m02() * b.x() - A.m00() * b.z() + AB.m01();
    AB.m02() = A.m00() * b.y() - A.m01() * b.x() + AB.m02();
    AB.m10() = A.m11() * b.z() - A.m12() * b.y() + AB.m10();
    AB.m11() = A.m12() * b.x() - A.m10() * b.z() + AB.m11();
    AB.m12() = A.m10() * b.y() - A.m11() * b.x() + AB.m12();
    AB.m20() = A.m21() * b.z() - A.m22() * b.y() + AB.m20();
    AB.m21() = A.m22() * b.x() - A.m20() * b.z() + AB.m21();
    AB.m22() = A.m20() * b.y() - A.m21() * b.x() + AB.m22();
  }
  static inline void ABT(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                         LA::AlignedMatrix3x3f &AB) {
    AB.m00() = A.m02() * b.y() - A.m01() * b.z();
    AB.m01() = A.m00() * b.z() - A.m02() * b.x();
    AB.m02() = A.m01() * b.x() - A.m00() * b.y();
    AB.m10() = A.m12() * b.y() - A.m11() * b.z();
    AB.m11() = A.m10() * b.z() - A.m12() * b.x();
    AB.m12() = A.m11() * b.x() - A.m10() * b.y();
    AB.m20() = A.m22() * b.y() - A.m21() * b.z();
    AB.m21() = A.m20() * b.z() - A.m22() * b.x();
    AB.m22() = A.m21() * b.x() - A.m20() * b.y();
  }
  static inline void ABT(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                         LA::SymmetricMatrix3x3f &AB) {
    AB.m00() = A.m02() * b.y() - A.m01() * b.z();
    AB.m01() = A.m00() * b.z() - A.m02() * b.x();
    AB.m02() = A.m01() * b.x() - A.m00() * b.y();
    AB.m11() = A.m10() * b.z() - A.m12() * b.x();
    AB.m12() = A.m11() * b.x() - A.m10() * b.y();
    AB.m22() = A.m21() * b.x() - A.m20() * b.y();
  }
  static inline void ABTToUpper(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                                LA::AlignedMatrix3x3f &AB) {
    AB.m00() = A.m02() * b.y() - A.m01() * b.z();
    AB.m01() = A.m00() * b.z() - A.m02() * b.x();
    AB.m02() = A.m01() * b.x() - A.m00() * b.y();
    AB.m11() = A.m10() * b.z() - A.m12() * b.x();
    AB.m12() = A.m11() * b.x() - A.m10() * b.y();
    AB.m22() = A.m21() * b.x() - A.m20() * b.y();
  }
  static inline void AddABTTo(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                              LA::SymmetricMatrix3x3f &AB) {
    AB.m00() = A.m02() * b.y() - A.m01() * b.z() + AB.m00();
    AB.m01() = A.m00() * b.z() - A.m02() * b.x() + AB.m01();
    AB.m02() = A.m01() * b.x() - A.m00() * b.y() + AB.m02();
    AB.m11() = A.m10() * b.z() - A.m12() * b.x() + AB.m11();
    AB.m12() = A.m11() * b.x() - A.m10() * b.y() + AB.m12();
    AB.m22() = A.m21() * b.x() - A.m20() * b.y() + AB.m22();
  }
  static inline void AddABTToUpper(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                                   LA::AlignedMatrix3x3f &AB) {
    AB.m00() = A.m02() * b.y() - A.m01() * b.z() + AB.m00();
    AB.m01() = A.m00() * b.z() - A.m02() * b.x() + AB.m01();
    AB.m02() = A.m01() * b.x() - A.m00() * b.y() + AB.m02();
    AB.m11() = A.m10() * b.z() - A.m12() * b.x() + AB.m11();
    AB.m12() = A.m11() * b.x() - A.m10() * b.y() + AB.m12();
    AB.m22() = A.m21() * b.x() - A.m20() * b.y() + AB.m22();
  }
  static inline void AddABTTo03(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                                LA::SymmetricMatrix6x6f &AB) {
    AB.m03() = A.m02() * b.y() - A.m01() * b.z() + AB.m03();
    AB.m04() = A.m00() * b.z() - A.m02() * b.x() + AB.m04();
    AB.m05() = A.m01() * b.x() - A.m00() * b.y() + AB.m05();
    AB.m13() = A.m12() * b.y() - A.m11() * b.z() + AB.m13();
    AB.m14() = A.m10() * b.z() - A.m12() * b.x() + AB.m14();
    AB.m15() = A.m11() * b.x() - A.m10() * b.y() + AB.m15();
    AB.m23() = A.m22() * b.y() - A.m21() * b.z() + AB.m23();
    AB.m24() = A.m20() * b.z() - A.m22() * b.x() + AB.m24();
    AB.m25() = A.m21() * b.x() - A.m20() * b.y() + AB.m25();
  }
  static inline void AddABTTo33(const LA::AlignedMatrix3x3f &A, const LA::AlignedVector3f &b,
                                LA::SymmetricMatrix6x6f &AB) {
    AB.m33() = A.m02() * b.y() - A.m01() * b.z() + AB.m33();
    AB.m34() = A.m00() * b.z() - A.m02() * b.x() + AB.m34();
    AB.m35() = A.m01() * b.x() - A.m00() * b.y() + AB.m35();
    AB.m44() = A.m10() * b.z() - A.m12() * b.x() + AB.m44();
    AB.m45() = A.m11() * b.x() - A.m10() * b.y() + AB.m45();
    AB.m55() = A.m21() * b.x() - A.m20() * b.y() + AB.m55();
  }
};

class Rotation3D : public LA::AlignedMatrix3x3f {

 public:

  inline Rotation3D() {}
  inline Rotation3D(const LA::AlignedVector3f &w, const float eps) {
    SetRodrigues(w, eps);
  }
  inline Rotation3D(const Quaternion &q) { SetQuaternion(q); }
  inline Rotation3D(const LA::AlignedMatrix3x3f &R) {
    memcpy(this, &R, sizeof(Rotation3D));
    //MakeOrthogonal();
  }
  inline Rotation3D(const float R[3][3]) { Set(R); MakeOrthogonal(); }

  inline const xp128f& r_00_01_02_x() const { return m_00_01_02_r0(); }
  inline       xp128f& r_00_01_02_x()       { return m_00_01_02_r0(); }
  inline const xp128f& r_10_11_12_x() const { return m_10_11_12_r1(); }
  inline       xp128f& r_10_11_12_x()       { return m_10_11_12_r1(); }
  inline const xp128f& r_20_21_22_x() const { return m_20_21_22_r2(); }
  inline       xp128f& r_20_21_22_x()       { return m_20_21_22_r2(); }

  inline const float& r00() const { return m00(); }   inline float& r00() { return m00(); }
  inline const float& r01() const { return m01(); }   inline float& r01() { return m01(); }
  inline const float& r02() const { return m02(); }   inline float& r02() { return m02(); }
  inline const float& r10() const { return m10(); }   inline float& r10() { return m10(); }
  inline const float& r11() const { return m11(); }   inline float& r11() { return m11(); }
  inline const float& r12() const { return m12(); }   inline float& r12() { return m12(); }
  inline const float& r20() const { return m20(); }   inline float& r20() { return m20(); }
  inline const float& r21() const { return m21(); }   inline float& r21() { return m21(); }
  inline const float& r22() const { return m22(); }   inline float& r22() { return m22(); }
  
  inline LA::AlignedMatrix3x3f operator * (const LA::AlignedMatrix3x3f &M) const {
    LA::AlignedMatrix3x3f RM;
    LA::AlignedMatrix3x3f::AB(*this, M, RM);
    return RM;
  }
  inline Rotation3D operator * (const Rotation3D &Rb) const {
    Rotation3D Rab;
    AB(*this, Rb, Rab);
    return Rab;
  }
  inline Rotation3D operator / (const Rotation3D &Rb) const {
    Rotation3D RaRbI;
    ABT(*this, Rb, RaRbI);
    return RaRbI;
  }

  inline void MakeIdentity(const LA::AlignedVector3f *g = NULL) {
    if (g) {
      LA::AlignedVector3f rx, ry, rz;
      g->GetMinus(rz);
      rz.Normalize();
      if (fabs(rz.z()) < fabs(rz.x())) {
        rx.x() = -rz.y();
        rx.y() =  rz.x();
        rx.z() =  0.0f;
        rx.Normalize();
        ry.x() = -rz.x() * rz.z();
        ry.y() = -rz.y() * rz.z();
        ry.z() =  rz.x() * rz.x() + rz.y() * rz.y();
        ry.Normalize();
      } else {
        ry.x() =  0.0f;
        ry.y() =  rz.z();
        ry.z() = -rz.y();
        ry.Normalize();
        rx.x() = rz.z() * rz.z() + rz.y() * rz.y();
        rx.y() = -rz.x() * rz.y();
        rx.z() = -rz.x() * rz.z();
        rx.Normalize();
      }
      SetColumn0(rx);
      SetColumn1(ry);
      SetColumn2(rz);
      MakeOrthogonal();
    } else {
      LA::AlignedMatrix3x3f::MakeIdentity();
    }
  }
  inline void MakeOrthogonal() {
    Quaternion q;
    GetQuaternion(q);
    SetQuaternion(q);
  }

  inline void SetRodrigues(const LA::AlignedVector3f &w, const float eps) {
    const LA::AlignedVector3f w2 = w.xyzr() * w.xyzr();
    const float th2 = w2.Sum(), th = sqrtf(th2);
    if (th < eps) {
      Quaternion q;
      const float s = 1.0f / sqrtf(th2 + 4.0f);
      q.xyzw() = w.xyzr() * s;
      q.w() = s + s;
      SetQuaternion(q);
      return;
    }
    const float t1 = UT_SINF(th) / th, t2 = (1.0f - UT_COSF(th)) / th2, t3 = 1.0f - t2 * th2;
    const LA::AlignedVector3f t1w = w.xyzr() * t1;
    const LA::AlignedVector3f t2w2 = w2.xyzr() * t2;
    const float t2wx = t2 * w.x();
    const float t2wxy = t2wx * w.y();
    const float t2wxz = t2wx * w.z();
    const float t2wyz = t2 * w.y() * w.z();
    r00() = t3 + t2w2.x();    r01() = t2wxy + t1w.z();  r02() = t2wxz - t1w.y();
    r10() = t2wxy - t1w.z();  r11() = t3 + t2w2.y();    r12() = t2wyz + t1w.x();
    r20() = t2wxz + t1w.y();  r21() = t2wyz - t1w.x();  r22() = t3 + t2w2.z();
  }
  inline void SetRodriguesXY(const LA::Vector2f &w, const float eps) {
    const LA::Vector2f w2 = w * w;
    const float th2 = w2.Sum(), th = sqrtf(th2);
    if (th < eps) {
      Quaternion q;
      const float s = 1.0f / sqrtf(th2 + 4.0f);
      q.x() = w.x() * s;
      q.y() = w.y() * s;
      q.z() = 0.0f;
      q.w() = s + s;
      SetQuaternion(q);
      return;
    }
    const float t1 = UT_SINF(th) / th, t2 = (1.0f - UT_COSF(th)) / th2, t3 = 1.0f - t2 * th2;
    const LA::Vector2f t1w = w * t1;
    const LA::Vector2f t2w2 = w2 * t2;
    const float t2wx = t2 * w.x();
    const float t2wxy = t2wx * w.y();
    r00() = t3 + t2w2.x();  r01() = t2wxy;          r02() = -t1w.y();
    r10() = t2wxy;          r11() = t3 + t2w2.y();  r12() = t1w.x();
    r20() = t1w.y();        r21() = -t1w.x();       r22() = t3;
  }
  inline void GetRodrigues(LA::AlignedVector3f &w, const float eps) const {
    //AxisAngle kth;
    //GetAxisAngle(kth);
    //kth.GetRodrigues(w);
    const float tr = Trace(), cth = (tr - 1.0f) * 0.5f, th = UT_DOT_TO_ANGLE(cth);
    const float t = th < eps ? 0.5f : th / (UT_SINF(th) * 2.0f);
    w.x() = r12() - r21();
    w.y() = r20() - r02();
    w.z() = r01() - r10();
    w *= t;
  }
  inline void GetRodriguesXY(LA::Vector2f &w, const float eps) const {
    const float tr = Trace(), cth = (tr - 1.0f) * 0.5f, th = UT_DOT_TO_ANGLE(cth);
    const float t = th < eps ? 0.5f : th / (UT_SINF(th) * 2.0f);
    w.x() = (r12() - r21()) * t;
    w.y() = (r20() - r02()) * t;
  }
  inline float GetRodriguesZ(const float eps) const {
    const float tr = Trace(), cth = (tr - 1.0f) * 0.5f, th = UT_DOT_TO_ANGLE(cth);
    const float t = th < eps ? 0.5f : th / (UT_SINF(th) * 2.0f);
    return (r01() - r10()) * t;
  }
  inline LA::AlignedVector3f GetRodrigues(const float eps) const {
    LA::AlignedVector3f w;
    GetRodrigues(w, eps);
    return w;
  }
  inline LA::Vector2f GetRodriguesXY(const float eps) const {
    LA::Vector2f w;
    GetRodriguesXY(w, eps);
    return w;
  }
  static inline void GetRodriguesJacobian(const LA::AlignedVector3f &w, LA::AlignedMatrix3x3f &Jr,
                                          const float eps) {
    const LA::AlignedVector3f w2(w.xyzr() * w.xyzr());
    const float th2 = w2.Sum(), th = sqrtf(th2);
    if (th < eps) {
      //Jr.MakeIdentity();
      const SkewSymmetricMatrix S = w * (-0.5f);
      S.Get(Jr);
      Jr.IncreaseDiagonal(1.0f);
      return;
    }
    const float th2I = 1.0f / th2;
    const float t1 = (1.0f - UT_COSF(th)) * th2I;
    const float t2 = (1.0f - UT_SINF(th) / th) * th2I;
    const float t3 = 1.0f - t2 * th2;
    const LA::AlignedVector3f t1w(w.xyzr() * t1);
    const LA::AlignedVector3f t2w2(w2.xyzr() * t2);
    const float t2wx = t2 * w.x();
    const float t2wxy = t2wx * w.y();
    const float t2wxz = t2wx * w.z();
    const float t2wyz = t2 * w.y() * w.z();
    Jr.m00() = t3 + t2w2.x(); Jr.m01() = t2wxy + t1w.z(); Jr.m02() = t2wxz - t1w.y();
    Jr.m10() = t2wxy - t1w.z(); Jr.m11() = t3 + t2w2.y(); Jr.m12() = t2wyz + t1w.x();
    Jr.m20() = t2wxz + t1w.y(); Jr.m21() = t2wyz - t1w.x(); Jr.m22() = t3 + t2w2.z();
  }
  static inline LA::AlignedMatrix3x3f GetRodriguesJacobian(const LA::AlignedVector3f &w,
                                                           const float eps) {
    LA::AlignedMatrix3x3f Jr;
    GetRodriguesJacobian(w, Jr, eps);
    return Jr;
  }
  static inline void GetRodriguesJacobianInverse(const LA::AlignedVector3f &w,
                                                 LA::AlignedMatrix3x3f &JrI,
                                                 const float eps) {
    const LA::AlignedVector3f w2(w.xyzr() * w.xyzr());
    const float th2 = w2.Sum(), th = sqrtf(th2);
    if (th < eps) {
      //JrI.MakeIdentity();
      const SkewSymmetricMatrix S = w * 0.5f;
      S.Get(JrI);
      JrI.IncreaseDiagonal(1.0f);
      return;
    }
    const float th2I = 1.0f / th2, t1 = -0.5f;
    const float t2 = th2I - (1.0f + UT_COSF(th)) / (2.0f * th * UT_SINF(th)), t3 = 1.0f - t2 * th2;
    const LA::AlignedVector3f t1w(w.xyzr() * t1);
    const LA::AlignedVector3f t2w2(w2.xyzr() * t2);
    const float t2wx = t2 * w.x();
    const float t2wxy = t2wx * w.y();
    const float t2wxz = t2wx * w.z();
    const float t2wyz = t2 * w.y() * w.z();
    JrI.m00() = t3 + t2w2.x();    JrI.m01() = t2wxy + t1w.z();  JrI.m02() = t2wxz - t1w.y();
    JrI.m10() = t2wxy - t1w.z();  JrI.m11() = t3 + t2w2.y();    JrI.m12() = t2wyz + t1w.x();
    JrI.m20() = t2wxz + t1w.y();  JrI.m21() = t2wyz - t1w.x();  JrI.m22() = t3 + t2w2.z();
  }
  static inline void GetRodriguesJacobianInverseXY(const LA::AlignedVector3f &w,
                                                   LA::AlignedMatrix2x3f &JrI,
                                                   const float eps) {
    const float th2 = w.SquaredLength(), th = sqrtf(th2);
    if (th < eps) {
      //JrI.MakeZero();
      //JrI.m00() = JrI.m11() = 1.0f;
      const SkewSymmetricMatrix S = w * 0.5f;
      JrI.m00() = 1.0f;   JrI.m01() = -S.z(); JrI.m02() = S.y();
      JrI.m10() = S.z();  JrI.m11() = 1.0f;   JrI.m12() = -S.x();
      return;
    }
    const float th2I = 1.0f / th2, t1 = -0.5f;
    const float t2 = th2I - (1.0f + UT_COSF(th)) / (2.0f * th * UT_SINF(th)), t3 = 1.0f - t2 * th2;
    const LA::AlignedVector3f t1w(w.xyzr() * t1);
    const float t2wx = t2 * w.x();
    const float t2wxx = t2wx * w.x();
    const float t2wxy = t2wx * w.y();
    const float t2wxz = t2wx * w.z();
    const float t2wy = t2 * w.y();
    const float t2wyy = t2wy * w.y();
    const float t2wyz = t2wy * w.z();
    JrI.m00() = t3 + t2wxx;       JrI.m01() = t2wxy + t1w.z();  JrI.m02() = t2wxz - t1w.y();
    JrI.m10() = t2wxy - t1w.z();  JrI.m11() = t3 + t2wyy;       JrI.m12() = t2wyz + t1w.x();
  }
  static inline LA::AlignedMatrix3x3f GetRodriguesJacobianInverse(const LA::AlignedVector3f &w,
                                                                  const float eps) {
    LA::AlignedMatrix3x3f JrI;
    GetRodriguesJacobianInverse(w, JrI, eps);
    return JrI;
  }

  inline void SetAxisAngle(const AxisAngle &kth) {
    const float cth = UT_COSF(kth.w()), sth2 = 1.0f - cth * cth, sth1 = sqrtf(sth2);
    const float sth = kth.w() >= 0.0f ? sth1 : -sth1;
    const LA::AlignedVector3f tk = kth.xyzw() * (1 - cth);
    const LA::AlignedVector3f tk2 = tk.xyzr() * kth.xyzw();
    const float tkxy = tk.x() * kth.y(), tkxz = tk.x() * kth.z(), tkyz = tk.y() * kth.z();
    const LA::AlignedVector3f sthk = kth.xyzw() * sth;
    r00() = cth + tk2.x();    r01() = tkxy + sthk.z();  r02() = tkxz - sthk.y();
    r10() = tkxy - sthk.z();  r11() = cth + tk2.y();    r12() = tkyz + sthk.x();
    r20() = tkxz + sthk.y();  r21() = tkyz - sthk.x();  r22() = cth + tk2.z();
  }
  inline void GetAxisAngle(AxisAngle &kth) const {
    const float tr = Trace(), cth = (tr - 1.0f) * 0.5f/*, th = UT_ACOSF(cth)*/;
    //const bool large = th >= UT_PI, small = th <= 0.0f;
    //const bool large = cth <= -1.0f, small = cth >= 1.0f;
    //const float _cth = UT_COSF(th);
    //const bool large = _cth <= -1.0f, small = _cth >= 1.0f;
    const bool large = cth < FLT_EPSILON - 1.0f, small = FLT_EPSILON + cth > 1.0f;
    if (!large && !small) {
      kth.x() = r12() - r21();
      kth.y() = r20() - r02();
      kth.z() = r01() - r10();
      kth.w() = UT_ACOSF(cth);
      kth.Normalize();
    } else if (large) {
      if (r00() >= r11() && r00() >= r22()) {
        kth.x() = -sqrtf(r00() - r11() - r22() + 1.0f) * 0.5f;
        kth.w() = 0.5f / kth.x();
        kth.y() = r01() * kth.w();
        kth.z() = r02() * kth.w();
      } else if (r11() >= r00() && r11() >= r22()) {
        kth.y() = -sqrtf(r11() - r00() - r22() + 1.0f) * 0.5f;
        kth.w()  = 0.5f / kth.y();
        kth.x() = r01() * kth.w();
        kth.z() = r12() * kth.w();
      } else if (r22() >= r00() && r22() >= r11()) {
        kth.z() = -sqrtf(r22() - r00() - r11() + 1.0f) * 0.5f;
        kth.w() = 0.5f / kth.z();
        kth.x() = r02() * kth.w();
        kth.y() = r12() * kth.w();
      }
      kth.w() = UT_PI;
    } else if (small)
      kth.MakeIdentity();
  }
  inline AxisAngle GetAxisAngle() const { AxisAngle kth; GetAxisAngle(kth); return kth; }
  static inline float GetAngle(const Rotation3D &R1, const Rotation3D &R2) {
    // TODO (yanghongtian) : computation order
    const float tr = (R1.r_00_01_02_x() * R2.r_00_01_02_x() +
                      R1.r_10_11_12_x() * R2.r_10_11_12_x() +
                      R1.r_20_21_22_x() * R2.r_20_21_22_x()).vsum_012();
    const float d = (tr - 1.0f) * 0.5f;
    return UT_DOT_TO_ANGLE(d);
  }

  inline void SetQuaternion(const Quaternion &q) {
    const xp128f t1 = q.xyzw() * q.x();
    const xp128f t2 = q.xyzw() * q.y();
    const float qzz = q.z() * q.z(), qzw = q.z() * q.w();

    r00() = t2[1] + qzz;
    r01() = t1[1] + qzw;
    r02() = t1[2] - t2[3];
    r10() = t1[1] - qzw;
    r11() = t1[0] + qzz;
    r12() = t2[2] + t1[3];
    r20() = t1[2] + t2[3];
    r21() = t2[2] - t1[3];
    r22() = t1[0] + t2[1];

    r_00_01_02_x() += r_00_01_02_x();  r00() = -r00() + 1.0f;
    r_10_11_12_x() += r_10_11_12_x();  r11() = -r11() + 1.0f;
    r_20_21_22_x() += r_20_21_22_x();  r22() = -r22() + 1.0f;
  }
  inline void GetQuaternion(Quaternion &q) const {
    q.w() = r00() + r11() + r22();
    //if (q.w() > r00() && q.w() > r11() && q.w() > r22())
    if (q.w() > 0.0f) {
      q.w() = sqrtf(q.w() + 1) * 0.5f;
      q.z() = 0.25f / q.w();
      q.x() = (r12() - r21()) * q.z();
      q.y() = (r20() - r02()) * q.z();
      q.z() = (r01() - r10()) * q.z();
    } else if (r00() > r11() && r00() > r22()) {
      q.x() = sqrtf(r00() + r00() - q.w() + 1) * 0.5f;
      q.w() = 0.25f / q.x();
      q.y() = (r01() + r10()) * q.w();
      q.z() = (r02() + r20()) * q.w();
      q.w() = (r12() - r21()) * q.w();
    } else if (r11() > r22()) {
      q.y() = sqrtf(r11() + r11() - q.w() + 1) * 0.5f;
      q.w() = 0.25f / q.y();
      q.x() = (r01() + r10()) * q.w();
      q.z() = (r12() + r21()) * q.w();
      q.w() = (r20() - r02()) * q.w();
    } else {
      q.z() = sqrtf(r22() + r22() - q.w() + 1) * 0.5f;
      q.w() = 0.25f / q.z();
      q.x() = (r02() + r20()) * q.w();
      q.y() = (r12() + r21()) * q.w();
      q.w() = (r01() - r10()) * q.w();
    }
    q.Normalize();
  }
  inline Quaternion GetQuaternion() const { Quaternion q; GetQuaternion(q); return q; }

  inline void SetEulerAngleX(const float th) {
    //Rx = [1, 0, 0; 0, cx, -sx; 0, sx, cx]
    //R = Rx^T = [1, 0, 0; 0, cx, sx; 0, -sx, cx]
    MakeIdentity();
    r11() = r22() = UT_COSF(th);
    r12() = UT_SINF(th);
    r21() = -r12();
  }
  inline void SetEulerAngleY(const float th) {
    //Ry = [cy, 0, sy; 0, 1, 0; -sy, 0, cy]
    //R = Ry^T = [cy, 0, -sy; 0, 1, 0; sy, 0, cy]
    MakeIdentity();
    r00() = r22() = UT_COSF(th);
    r20() = UT_SINF(th);
    r02() = -r20();
  }
  inline void SetEulerAngleZ(const float th) {
    //Rz = [cz, -sz, 0; sz, cz, 0; 0, 0, 1]
    //R = Rz^T = [cz, sz, 0; -sz, cz, 0; 0, 0, 1]
    MakeIdentity();
    r00() = r11() = UT_COSF(th);
    r01() = UT_SINF(th);
    r10() = -r01();
  }
  inline void SetEulerAnglesZXY(const float yaw, const float pitch, const float roll) {
    // R(c->w) = Rz(yaw) * Rx(pitch) * Ry(roll)
    // R = R(c->w)^T
    const float cx = UT_COSF(pitch), sx = UT_SINF(pitch);
    const float cy = UT_COSF(roll), sy = UT_SINF(roll);
    const float cz = UT_COSF(yaw), sz = UT_SINF(yaw);
    const float cycz = cy * cz, sxsy = sx * sy, cysz = cy * sz;
    //r00() = cycz - sxsy * sz;
    //r10() = -cx * sz;
    //r20() = sy * cz + sx * cysz;
    //r01() = cysz + sxsy * cz;
    //r11() = cx * cz;
    //r21() = sy * sz - sx * cycz;
    //r02() = -cx * sy;
    //r12() = sx;
    //r22() = cx * cy;
    r00() = cycz - sxsy * sz;
    r01() = cysz + sxsy * cz;
    r02() = -cx * sy;
    r10() = -cx * sz;
    r11() = cx * cz;
    r12() = sx;
    r20() = sy * cz + sx * cysz;
    r21() = sy * sz - sx * cycz;
    r22() = cx * cy;
  }
  inline void SetEulerAnglesZYX(const float yaw, const float pitch, const float roll) {
    // R(c->w) = Rz(yaw) * Ry(pitch) * Rx(roll)
    // R = R(c->w)^T
    const float cx = UT_COSF(roll), sx = UT_SINF(roll);
    const float cy = UT_COSF(pitch), sy = UT_SINF(pitch);
    const float cz = UT_COSF(yaw), sz = UT_SINF(yaw);
    const float sxsy = sx * sy, cxsz = cx * sz, cxcz = cx * cz;
    //r00() = cy * cz;
    //r10() = sxsy * cz - cxsz;
    //r20() = sx * sz + cxcz * sy;
    //r01() = cy * sz;
    //r11() = cxcz + sxsy * sz;
    //r21() = cxsz * sy - sx * cz;
    //r02() = -sy;
    //r12() = sx * cy;
    //r22() = cx * cy;
    r00() = cy * cz;
    r01() = cy * sz;
    r02() = -sy;
    r10() = sxsy * cz - cxsz;
    r11() = cxcz + sxsy * sz;
    r12() = sx * cy;
    r20() = sx * sz + cxcz * sy;
    r21() = cxsz * sy - sx * cz;
    r22() = cx * cy;
  }

  inline void GetTranspose(Rotation3D &RT) const { LA::AlignedMatrix3x3f::GetTranspose(RT); }
  inline Rotation3D GetTranspose() const { Rotation3D RT; GetTranspose(RT); return RT; }

  inline void GetGravity(LA::AlignedVector3f &g) const { GetColumn2(g); g.MakeMinus(); }
  inline LA::AlignedVector3f GetGravity() const {
    LA::AlignedVector3f g;
    GetGravity(g);
    return g;
  }

  inline void Apply(const LA::AlignedVector3f &X, LA::AlignedVector3f &RX) const {
    Apply(X.xyzr(), RX.xyzr());
  }
  inline void Apply(const xp128f &X, xp128f &RX) const {
    RX[0] = (r_00_01_02_x() * X).vsum_012();
    RX[1] = (r_10_11_12_x() * X).vsum_012();
    RX[2] = (r_20_21_22_x() * X).vsum_012();
  }
  inline void Apply(const Point2D &x, Point2D &Rx) const {
    Rx.y() = 1.0f / (r20() * x.x() + r21() * x.y() + r22());
    Rx.x() = (r00() * x.x() + r01() * x.y() + r02()) * Rx.y();
    Rx.y() = (r10() * x.x() + r11() * x.y() + r12()) * Rx.y();
  }
  inline void Apply(const Point2D &x, LA::Vector3f &Rx) const {
    Rx.x() = r00() * x.x() + r01() * x.y() + r02();
    Rx.y() = r10() * x.x() + r11() * x.y() + r12();
    Rx.z() = r20() * x.x() + r21() * x.y() + r22();
  }
  inline void Apply(const Point2D &x, Point2D &Rx, LA::AlignedMatrix2x2f &Jx) const {
    const float d = 1.0f / (r20() * x.x() + r21() * x.y() + r22());
    Rx.x() = (r00() * x.x() + r01() * x.y() + r02()) * d;
    Rx.y() = (r10() * x.x() + r11() * x.y() + r12()) * d;
    Jx.m00() = r00() - Rx.x() * r20();
    Jx.m01() = r01() - Rx.x() * r21();
    Jx.m10() = r10() - Rx.y() * r20();
    Jx.m11() = r11() - Rx.y() * r21();
    Jx *= d;
  }
  inline LA::AlignedVector3f GetApplied(const LA::AlignedVector3f &X) const {
    LA::AlignedVector3f RX; Apply(X, RX); return RX;
  }
  inline Point2D GetApplied(const Point2D &x) const { Point2D Rx; Apply(x, Rx); return Rx; }
  inline Point2D GetApplied(const Point2D &x, LA::AlignedMatrix2x2f &Jx) const {
    Point2D Rx; Apply(x, Rx, Jx); return Rx;
  }
  inline void ApplyInversely(const LA::AlignedVector3f &X, LA::AlignedVector3f &RTX) const {
    // TODO (yanghongtian) : computation order
    RTX.xyzr() = r_00_01_02_x() * X.x() +
                 r_10_11_12_x() * X.y() +
                 r_20_21_22_x() * X.z();
  }
  inline LA::AlignedVector3f GetAppliedInversely(const LA::AlignedVector3f &X) const {
    LA::AlignedVector3f RTX;
    ApplyInversely(X, RTX);
    return RTX;
  }
  inline void ApplyInversely(const LA::AlignedVector3f &X, Point3D &RTX) const {
    // TODO (yanghongtian) : computation order
    RTX.xyzw() = r_00_01_02_x() * X.x() +
                 r_10_11_12_x() * X.y() +
                 r_20_21_22_x() * X.z();
    RTX.w() = 1.0f;
  }

  inline float DotOrientation(const Rotation3D &R) const {
    const float d = r_20_21_22_x().vdot012(R.r_20_21_22_x());
    return UT_CLAMP(d, -1.0f, 1.0f);
  }

  inline void Random(const float thMax) {
    AxisAngle kth;
    kth.Random(thMax);
    SetAxisAngle(kth);
  }
  static inline Rotation3D GetRandom(const float thMax) {
    Rotation3D R;
    R.Random(thMax);
    return R;
  }

  inline bool AssertEqual(const Rotation3D &R, const int verbose = 1, const std::string str = "",
                          const float eps = 0.001745329252f) const {
    const float dr = Rotation3D::GetAngle(*this, R);
    //const float eps = 0.1f * UT_FACTOR_DEG_TO_RAD;
    if ((eps < 0.0f && UT::VectorAssertEqual(&r00(), &R.r00(), 12, verbose, str, eps)) ||
        (eps >= 0.0f && UT::AssertEqual(dr, 0.0f, verbose, str, eps))) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      R.Print(verbose > 1);
    }
    return false;
  }

  inline bool AssertOrthogonal(const int verbose = 1, const std::string str = "") const {
    if (!UT::AssertEqual(Determinant(), 1.0f, verbose, str + "|R|")) {
      return false;
    }
    if (!AssertOrthogonalRows(verbose, str)) {
      return false;
    }
    const Rotation3D RT = GetTranspose();
    if (!RT.AssertOrthogonalRows(verbose, str + ".T")) {
      return false;
    }
    return true;
  }
  inline bool AssertOrthogonalRows(const int verbose = 1, const std::string str = "",
                                   const float eps = 0.001745329252f) const {
    //const float eps = 0.1f * UT_FACTOR_DEG_TO_RAD;
    const float d00 = r_00_01_02_x().vdot012(r_00_01_02_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(d00), 0.0f, verbose, str + ".rxTrx", eps)) {
      return false;
    }
    const float d01 = r_00_01_02_x().vdot012(r_10_11_12_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(d01) - UT_PI_2, 0.0f, verbose, str + ".rxTry", eps)) {
      return false;
    }
    const float d02 = r_00_01_02_x().vdot012(r_20_21_22_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(d02) - UT_PI_2, 0.0f, verbose, str + ".rxTrz", eps)) {
      return false;
    }
    const float d11 = r_10_11_12_x().vdot012(r_10_11_12_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(d11), 0.0f, verbose, str + ".ryTry", eps)) {
      return false;
    }
    const float d12 = r_10_11_12_x().vdot012(r_20_21_22_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(d12) - UT_PI_2, 0.0f, verbose, str + ".ryTrz", eps)) {
      return false;
    }
    const float d22 = r_20_21_22_x().vdot012(r_20_21_22_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(d22), 0.0f, verbose, str + ".rzTrz", eps)) {
      return false;
    }

    xp128f c = r_10_11_12_x().vcross012(r_20_21_22_x());
    const float dc0 = c.vdot012(r_00_01_02_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(dc0), 0.0f, verbose, str + ".rx", eps)) {
      return false;
    }
    c = r_20_21_22_x().vcross012(r_00_01_02_x());
    const float dc1 = c.vdot012(r_10_11_12_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(dc1), 0.0f, verbose, str + ".ry", eps)) {
      return false;
    }
    c = r_00_01_02_x().vcross012(r_10_11_12_x());
    const float dc2 = c.vdot012(r_20_21_22_x());
    if (!UT::AssertEqual(UT_DOT_TO_ANGLE(dc2), 0.0f, verbose, str + ".rz", eps)) {
      return false;
    }
    return true;
  }

  static inline void AB(const Rotation3D &Ra, const Rotation3D &Rb, Rotation3D &Rab) {
    const Rotation3D RbT = Rb.GetTranspose();
    ABT(Ra, RbT, Rab);
  }
  static inline void ABT(const Rotation3D &Ra, const Rotation3D &Rb, Rotation3D &RaRbT) {
    LA::AlignedMatrix3x3f::ABT(Ra, Rb, RaRbT);
    RaRbT.MakeOrthogonal();
  }
  static inline void ATB(const Rotation3D &Ra, const Rotation3D &Rb, Rotation3D &RaTRb) {
    const Rotation3D RaT = Ra.GetTranspose();
    AB(RaT, Rb, RaTRb);
  }
};

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Geometry>
class EigenAxisAngle : public Eigen::AngleAxisf {
 public:
  inline EigenAxisAngle() : Eigen::AngleAxisf() {}
  inline EigenAxisAngle(const Eigen::AngleAxisf &e_th) : Eigen::AngleAxisf(e_th) {}
  inline EigenAxisAngle(const AxisAngle &kth) : Eigen::AngleAxisf() { Set(EigenVector3f(kth.x(), kth.y(), kth.z()), kth.w()); }
  inline void operator = (const Eigen::AngleAxisf &e_kth) { *((Eigen::AngleAxisf *) this) = e_kth; }
  inline AxisAngle GetAxisAngle() const {
    return AxisAngle(GetAxis().GetAlignedVector3f().GetNormalized(), GetAngle());
  }
  inline void Set(const EigenVector3f &e_k, const float th) {
    axis() = e_k;
    angle() = th;
#ifdef CFG_DEBUG
    UT::AssertEqual(e_k.SquaredLength(), 1.0f);
#endif
  }
  inline EigenVector3f GetAxis() const { return EigenVector3f(axis()); }
  inline float GetAngle() const { return angle(); }
  inline void MakeIdentity() { axis() = Eigen::Vector3f(1.0f, 0.0f, 0.0f); angle() = 0.0f; }
  inline void SetRodrigues(const EigenVector3f &e_w) {
    const float th = sqrtf(e_w.SquaredLength());
    if (th < FLT_EPSILON) {
      MakeIdentity();
    } else {
      axis() = e_w / th;
      angle() = th;
    }
#ifdef CFG_DEBUG
    UT::AssertEqual(axis().squaredNorm(), 1.0f);
#endif
  }
  inline void GetRodrigues(EigenVector3f &e_w) const {
    const float e_th = angle() < UT_PI ? angle() : angle() - UT_2PI;
    e_w = axis() * e_th;
  }
  inline EigenVector3f GetRodrigues() const { EigenVector3f e_w; GetRodrigues(e_w); return e_w; }
  inline void Print(const bool e = false) const { GetAxisAngle().Print(e); }
  static inline EigenAxisAngle GetRandom(const float thMax) { return EigenAxisAngle(AxisAngle::GetRandom(thMax)); }
  inline bool AssertEqual(const EigenAxisAngle &e_kth, const int verbose = 1) const { return GetAxisAngle().AssertEqual(e_kth.GetAxisAngle(), verbose); }
};
class EigenQuaternion : public EigenVector4f {
 public:
  inline EigenQuaternion() : EigenVector4f() {}
  inline EigenQuaternion(const EigenVector4f &e_q) : EigenVector4f(e_q) {}
  inline EigenQuaternion(const Quaternion &q) : EigenVector4f(q.x(), q.y(), q.z(), q.w()) {}
  inline EigenQuaternion(const Eigen::Quaternionf &e_q) :
                         EigenVector4f(e_q.x(), e_q.y(), e_q.z(), e_q.w()) {}
  inline Quaternion GetQuaternion() const { return GetAlignedVector4f(); }
  inline Eigen::Quaternionf GetEigenQuaternion() const {
    Eigen::Quaternionf e_q;
    e_q.x() = x();
    e_q.y() = y();
    e_q.z() = z();
    e_q.w() = w();
    return e_q;
  }
  inline void MakeIdentity() { x() = 0.0f; y() = 0.0f; z() = 0.0f; w() = 1.0f; }
  inline void Normalize() {
    const float s = 1.0f / ::sqrtf(SquaredLength());
    (*this) *= w() > 0.0f ? s : -s;
  }
  inline void SetRodrigues(const EigenVector3f &e_w, const float eps) {
    const float th = sqrtf(e_w.SquaredLength());
    if (th < eps) {
      block<3, 1>(0, 0) = e_w * 0.5f;
      w() = 1.0f;
      Normalize();
    } else {
      EigenAxisAngle e_kth;
      e_kth.SetRodrigues(e_w);
      SetAxisAngle(e_kth);
    }
  }
  inline void GetRodrigues(EigenVector3f &e_w) const {
    EigenAxisAngle e_kth;
    GetAxisAngle(e_kth);
    e_kth.GetRodrigues(e_w);
  }
  inline void SetAxisAngle(const EigenAxisAngle &e_kth) {
    const EigenVector3f e_k = e_kth.GetAxis();
#ifdef CFG_DEBUG
    UT::AssertEqual(e_k.SquaredLength(), 1.0f);
#endif
    const float thh = e_kth.GetAngle() * 0.5f, sthh = sinf(thh), cthh = cosf(thh);
    x() = e_k.x() * sthh;
    y() = e_k.y() * sthh;
    z() = e_k.z() * sthh;
    w() = cthh;
  }
  inline void GetAxisAngle(EigenAxisAngle &e_kth) const {
    const EigenVector3f e_k(x(), y(), z());
    e_kth.Set(EigenVector3f(e_k / sqrtf(e_k.squaredNorm())), UT_ACOSF(w()) * 2.0f);
  }
  inline bool AssertEqual(const Quaternion &q, const int verbose = 1) const { return GetQuaternion().AssertEqual(q, verbose); }
  inline bool AssertEqual(const EigenQuaternion &e_q, const int verbose = 1) const { return GetQuaternion().AssertEqual(e_q.GetQuaternion(), verbose); }
};
class EigenSkewSymmetricMatrix : public EigenMatrix3x3f {
 public:
  inline EigenSkewSymmetricMatrix() : EigenMatrix3x3f() {}
  inline EigenSkewSymmetricMatrix(const Eigen::Vector3f &e_w) : EigenMatrix3x3f() { Set(EigenVector3f(e_w).GetAlignedVector3f()); }
  inline EigenSkewSymmetricMatrix(const SkewSymmetricMatrix &M) : EigenMatrix3x3f() { Set(M); }
  inline EigenSkewSymmetricMatrix(const Eigen::Matrix3f &e_M) : EigenMatrix3x3f(e_M) {}
  inline void Set(const LA::AlignedVector3f &w) {
    Eigen::Matrix3f &e_S = *this;
    e_S(0, 0) = 0.0f; e_S(0, 1) = -w.z(); e_S(0, 2) = w.y();
    e_S(1, 0) = w.z();  e_S(1, 1) = 0.0f; e_S(1, 2) = -w.x();
    e_S(2, 0) = -w.y(); e_S(2, 1) = w.x();  e_S(2, 2) = 0.0f;
  }
};
class EigenRotation3D : public EigenMatrix3x3f {
 public:
  inline EigenRotation3D() : EigenMatrix3x3f() {}
  inline EigenRotation3D(const Eigen::Matrix3f &e_R) : EigenMatrix3x3f(e_R) {}
  inline EigenRotation3D(const EigenMatrix3x3f &e_R) : EigenMatrix3x3f(e_R) {}
  inline EigenRotation3D(const EigenVector3f &e_w, const float eps) {
    SetRodrigues(e_w, eps);
  }
  inline EigenRotation3D(const EigenAxisAngle &e_kth) { SetAxisAngle(e_kth); }
  inline EigenRotation3D(const EigenQuaternion &e_q) { SetQuaternion(e_q); }
  inline EigenRotation3D(const Rotation3D &R) : EigenMatrix3x3f(R) {}
  inline EigenRotation3D operator * (const EigenRotation3D &e_Rb) const {
    const Eigen::Matrix3f &e_Ra = *this;
    EigenRotation3D e_Rab = EigenMatrix3x3f(e_Ra * e_Rb);
    e_Rab.MakeOrthogonal();
    return e_Rab;
  }
  inline Eigen::Matrix3f operator * (const Eigen::Matrix3f &e_M) const {
    const Eigen::Matrix3f &e_R = *this;
    return e_R * e_M;
  }
  inline Eigen::Vector3f operator * (const Eigen::Vector3f &e_X) const {
    const Eigen::Matrix3f &e_R = *this;
    return e_R * e_X; 
  }
  inline EigenRotation3D GetTranspose() const { return EigenRotation3D(transpose()); }
  inline void SetRodrigues(const EigenVector3f &e_w, const float eps) {
#if 0
    EigenAxisAngle e_kth;
    e_kth.SetRodrigues(e_w);
    SetAxisAngle(e_kth);
#else
    Rotation3D R;
    R.SetRodrigues(e_w.GetAlignedVector3f(), eps);
    *this = R;
#endif
  }
  inline void GetRodrigues(EigenVector3f &e_w, const float eps) const {
#if 0
    EigenAxisAngle e_kth;
    GetAxisAngle(e_kth);
    e_kth.GetRodrigues(e_w);
#else
    const Rotation3D R = GetAlignedMatrix3x3f();
    e_w = R.GetRodrigues(eps);
#endif
  }
  inline EigenVector3f GetRodrigues(const float eps) const {
    EigenVector3f e_w;
    GetRodrigues(e_w, eps);
    return e_w;
  }
  static inline EigenMatrix3x3f GetRodriguesJacobian(const EigenVector3f &e_w,
                                                     const float eps) {
    //return EigenMatrix3x3f(Rotation3D::GetRodriguesJacobian(e_w.GetAlignedVector3f()));
    const EigenSkewSymmetricMatrix e_S(e_w);
    const float th = sqrtf(e_w.SquaredLength());
    if (th < eps) {
      return EigenMatrix3x3f(EigenMatrix3x3f::Identity() - 0.5f * e_S);
    } else {
      const float th2 = th * th;
      return EigenMatrix3x3f(EigenMatrix3x3f::Identity()
                          - (1.0f - cosf(th)) / th2 * e_S
                          + (th - sinf(th)) / (th2 * th) * e_S * e_S);
    }
  }
  static inline EigenMatrix3x3f GetRodriguesJacobianInverse(const EigenVector3f &e_w,
                                                            const float eps) {
    //return EigenMatrix3x3f(Rotation3D::GetRodriguesJacobianInverse(e_w.GetAlignedVector3f()));
    const EigenSkewSymmetricMatrix e_S(e_w);
    const float th = sqrtf(e_w.SquaredLength());
    if (th < eps) {
      return EigenMatrix3x3f(EigenMatrix3x3f::Identity() + 0.5f * e_S);
    } else {
      return EigenMatrix3x3f(EigenMatrix3x3f::Identity()
                           + 0.5f * e_S + (1.0f / (th * th)
                           - (1.0f + cosf(th)) / (2.0f * th * sinf(th))) * e_S * e_S);
    }
  }
  inline void operator = (const Eigen::Matrix3f &e_R) { *((EigenMatrix3x3f *) this) = e_R; }
  inline void SetAxisAngle(const EigenAxisAngle &e_kth) { *this = e_kth.toRotationMatrix().transpose(); }
  inline void GetAxisAngle(EigenAxisAngle &e_kth) const { e_kth.fromRotationMatrix(transpose()); }
  inline void SetQuaternion(const EigenQuaternion &e_q) {
    //*this = e_q.GetEigenQuaternion().toRotationMatrix().transpose();
    Rotation3D R;
    R.SetQuaternion(e_q.GetQuaternion());
    *this = R;
  }
  inline void GetQuaternion(EigenQuaternion &e_q) const {
    //EigenAxisAngle e_kth;
    //e_kth.fromRotationMatrix(transpose());
    //e_q.SetAxisAngle(e_kth);
    const Rotation3D R = GetAlignedMatrix3x3f();
    e_q = R.GetQuaternion();
  }
  inline void MakeOrthogonal() {
    EigenQuaternion q;
    GetQuaternion(q);
    SetQuaternion(q);
  }
};
#endif
#endif
