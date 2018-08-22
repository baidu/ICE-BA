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
#ifndef _RIGID_H_
#define _RIGID_H_

#include "Rotation.h"
#include "Matrix4x4.h"

class Rigid3D : public Rotation3D {

 public:

  class Row : public LA::AlignedVector3f {
   public:
    inline Row() {}
    inline Row(const LA::AlignedVector3f &v) : LA::AlignedVector3f(v) {}
    inline Row(const xp128f &v) : LA::AlignedVector3f(v) {}
    inline const xp128f& r0_r1_r2_t() const { return v012r(); }
    inline       xp128f& r0_r1_r2_t()       { return v012r(); }
    inline const float& r0() const { return v0(); }   inline float& r0() { return v0(); }
    inline const float& r1() const { return v1(); }   inline float& r1() { return v1(); }
    inline const float& r2() const { return v2(); }   inline float& r2() { return v2(); }
    inline const float& t () const { return r (); }   inline float& t () { return r (); }
    inline Row operator / (const Rigid3D &Tb) const { Row TaTbI; Rigid3D::ABI(*this, Tb, TaTbI); return TaTbI; }
  };

 public:

  inline Rigid3D() {}
  inline Rigid3D(const Rotation3D &R, const Point3D &p) {
    memcpy(this, &R, sizeof(Rotation3D));
    SetPosition(p);
  }
  inline Rigid3D(const Rigid3D &T, const LA::AlignedVector3f *dp, const LA::AlignedVector3f *dr,
                 const float eps) {
    Point3D p;
    T.GetPosition(p);
    if (dp) {
      p += *dp;
    }
    if (dr) {
      const Rotation3D dR(*dr, eps);
      Rotation3D::AB(T, dR, *this);
    } else {
      *this = T;
    }
    SetPosition(p);
  }
  inline Rigid3D(const float T[3][4]) { Set(T); }

  inline const xp128f& r00_r01_r02_tx() const { return r_00_01_02_x(); }
  inline       xp128f& r00_r01_r02_tx()       { return r_00_01_02_x(); }
  inline const xp128f& r10_r11_r12_ty() const { return r_10_11_12_x(); }
  inline       xp128f& r10_r11_r12_ty()       { return r_10_11_12_x(); }
  inline const xp128f& r20_r21_r22_tz() const { return r_20_21_22_x(); }
  inline       xp128f& r20_r21_r22_tz()       { return r_20_21_22_x(); }
  inline const float& tx() const { return r0(); } inline float& tx() { return r0(); }
  inline const float& ty() const { return r1(); } inline float& ty() { return r1(); }
  inline const float& tz() const { return r2(); } inline float& tz() { return r2(); }

  inline void operator = (const Rigid3D &T) {
    memcpy(this, &T, sizeof(Rigid3D));
  }
  inline void operator = (const Rotation3D &R) {
    memcpy(this, &R, sizeof(Rotation3D));
    tx() = ty() = tz() = 0.0f;
  }
  inline bool operator == (const Rigid3D &M) const {
    // TODO (yanghongtian) : rooms for optimization
    return r00() == M.r00() && r01() == M.r01() && r02() == M.r02() && tx() == M.tx() &&
           r10() == M.r10() && r11() == M.r11() && r12() == M.r12() && ty() == M.ty() &&
           r20() == M.r20() && r21() == M.r21() && r22() == M.r22() && tz() == M.tz();
  }
  inline Point3D operator * (const Point3D &X) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    Point3D TX;
    Apply(X, TX);
    return TX;
  }
  inline Rigid3D operator * (const Rigid3D &Tb) const {
    Rigid3D Tab;
    const Rotation3D RbT = Tb.LA::AlignedMatrix3x3f::GetTranspose();
    LA::AlignedMatrix3x3f::ABT(*this, RbT, Tab);
    Tab.MakeOrthogonal();
    Tab.SetTranslation(GetApplied(Tb.GetTranslation()));
    return Tab;
  }
  inline Rigid3D operator / (const Rigid3D &Tb) const {
    Rigid3D TaTbI;
    ABI(*this, Tb, TaTbI);
    return TaTbI;
  }

  inline void MakeIdentity(const LA::AlignedVector3f *g = NULL) {
    Rotation3D::MakeIdentity(g);
    tx() = ty() = tz() = 0.0f;
  }

  inline void Set(const float *T) {
    memcpy(this, T, sizeof(Rigid3D));
  }
  inline void Set(const double *T) {
    r00_r01_r02_tx().vset_all_lane(float(T[0]), float(T[1]), float(T[2]), float(T[3]));
    r10_r11_r12_ty().vset_all_lane(float(T[4]), float(T[5]), float(T[6]), float(T[7]));
    r20_r21_r22_tz().vset_all_lane(float(T[8]), float(T[9]), float(T[10]), float(T[11]));
  }
  inline void Set(const float T[3][4]) {
    memcpy(&r00(), T[0], 16);
    memcpy(&r10(), T[1], 16);
    memcpy(&r20(), T[2], 16);
  }
  inline void Set(const Quaternion &q, const Point3D &p) {
    SetRotation(q);
    SetPosition(p);
  }
  inline void Set(const Rotation3D &R, const LA::AlignedVector3f &t) {
    *this = R;
    SetTranslation(t);
  }
  inline void SetRowX(const Row &rx) { r_00_01_02_x() = rx.r0_r1_r2_t(); }
  inline void SetRowY(const Row &ry) { r_10_11_12_x() = ry.r0_r1_r2_t(); }
  inline void SetRowZ(const Row &rz) { r_20_21_22_x() = rz.r0_r1_r2_t(); }
  inline void SetRotation(const Quaternion &q) { SetQuaternion(q); }
  inline void SetTranslation(const LA::AlignedVector3f &t) {
    tx() = t.x();
    ty() = t.y();
    tz() = t.z();
  }
  inline void SetPosition(const LA::AlignedVector3f &p) { SetPosition(p.xyzr()); }
  inline void SetPosition(const xp128f &p) {
    tx() = -(r_00_01_02_x() * p).vsum_012();
    ty() = -(r_10_11_12_x() * p).vsum_012();
    tz() = -(r_20_21_22_x() * p).vsum_012();
  }
  inline void ScaleTranslation(const float s) { tx() *= s; ty() *= s; tz() *= s; }

  inline void Get(Quaternion &q, Point3D &p) const { GetRotation(q); GetPosition(p); }
  inline void Get(float *q, float *p) const {
    Quaternion _q;
    Point3D _p;
    Get(_q, _p);
    _q.Get(q);
    _p.Get(p);
  }
  inline void Get(float T[3][4]) const {
    memcpy(T[0], &r00(), 16);
    memcpy(T[1], &r10(), 16);
    memcpy(T[2], &r20(), 16);
  }
  inline void GetTranslation(LA::AlignedVector3f &t) const {
    t.xyzr().vset_all_lane(tx(), ty(), tz(), 0.0f);
  }
  inline LA::AlignedVector3f GetTranslation() const {
    return LA::AlignedVector3f(tx(), ty(), tz());
  }
  inline Row GetRowX() const { return Row(r_00_01_02_x()); }
  inline Row GetRowY() const { return Row(r_10_11_12_x()); }
  inline Row GetRowZ() const { return Row(r_20_21_22_x()); }
  inline void GetRotation(Quaternion &q) const { GetQuaternion(q); }
  inline void GetRotationTranspose(Rotation3D &RT) const { Rotation3D::GetTranspose(RT); }
  inline Rotation3D GetRotationTranspose() const { return Rotation3D::GetTranspose(); }
  inline void GetPosition(Point3D &p) const { GetPosition(p.xyzw()); p.w() = 1.0f; }
  inline void GetPosition(LA::AlignedVector3f &p) const { GetPosition(p.xyzr()); }
  inline void GetPosition(xp128f &p) const {
    // TODO (yanghongtian) : computation order
    p = r_00_01_02_x() * (-tx()) +
        r_10_11_12_x() * (-ty()) +
        r_20_21_22_x() * (-tz());
  }
  inline Point3D GetPosition() const { Point3D p; GetPosition(p); return p; }
  inline float GetPositionZ() const {
    return -(r02() * tx() + r12() * ty() + r22() * tz());
  }
  inline void GetTranspose(LA::AlignedMatrix4x4f &MT) const {
    MT.m00() = r00(); MT.m10() = r01(); MT.m20() = r02(); MT.m30() = tx();
    MT.m01() = r10(); MT.m11() = r11(); MT.m21() = r12(); MT.m31() = ty();
    MT.m02() = r20(); MT.m12() = r21(); MT.m22() = r22(); MT.m32() = tz();
    MT.m03() = 0.0f;  MT.m13() = 0.0f;  MT.m23() = 0.0f;  MT.m33() = 1.0f;
  }
  inline void Inverse() {
    const Point3D p = GetPosition();
    Transpose();
    SetTranslation(p);
  }
  inline void GetInverse(Rigid3D &T) const {
    Rotation3D::GetTranspose(T);
    T.SetPosition(GetTranslation());
  }
  inline Rigid3D GetInverse() const { Rigid3D T; GetInverse(T); return T; }

  inline void Apply(const Point3D &X, LA::AlignedVector3f &TX) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    TX.x() = (r00_r01_r02_tx() * X.xyzw()).vsum_all();
    TX.y() = (r10_r11_r12_ty() * X.xyzw()).vsum_all();
    TX.z() = (r20_r21_r22_tz() * X.xyzw()).vsum_all();
  }
  inline bool Apply(const Point3D &X, Point2D &x) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    x.y() = (r20_r21_r22_tz() * X.xyzw()).vsum_all();
    if (x.y() < 0.0f) {
      return false;
    }
    x.y() = 1.0f / x.y();
    x.x() = (r00_r01_r02_tx() * X.xyzw()).vsum_all() * x.y();
    x.y() = (r10_r11_r12_ty() * X.xyzw()).vsum_all() * x.y();
    return true;
  }
  inline bool Apply(const Point3D &X, Point2D &x, float &z) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    z = (r20_r21_r22_tz() * X.xyzw()).vsum_all();
    if (z < 0.0f) {
      return false;
    }
    x.y() = 1.0f / z;
    x.x() = (r00_r01_r02_tx() * X.xyzw()).vsum_all() * x.y();
    x.y() = (r10_r11_r12_ty() * X.xyzw()).vsum_all() * x.y();
    return true;
  }
  inline LA::AlignedVector3f GetApplied(const Point3D &X) const { LA::AlignedVector3f TX; Apply(X, TX); return TX; }
  inline float GetAppliedZ(const Point3D &X) const {
#ifdef CFG_DEBUG
    UT_ASSERT(X.w() == 1.0f);
#endif
    return (r20_r21_r22_tz() * X.xyzw()).vsum_all();
  }
  inline void ApplyRotation(const LA::AlignedVector3f &X, LA::AlignedVector3f &RX) const { Rotation3D::Apply(X, RX); }
  inline void ApplyRotation(const xp128f &X, xp128f &RX) const { Rotation3D::Apply(X, RX); }
  inline void ApplyRotation(const Point2D &x, LA::Vector3f &Rx) const { Rotation3D::Apply(x, Rx); }
  inline void ApplyRotationInversely(const LA::AlignedVector3f &X, LA::AlignedVector3f &RTX) const { Rotation3D::ApplyInversely(X, RTX); }
  inline LA::AlignedVector3f GetAppliedRotation(const LA::AlignedVector3f &X) const { return Rotation3D::GetApplied(X); }
  inline LA::AlignedVector3f GetAppliedRotationInversely(const LA::AlignedVector3f &X) const { return Rotation3D::GetAppliedInversely(X); }
  inline void ApplyInversely(Point3D &X) const {
    X.xyzw() = r_00_01_02_x() * (X.x() - tx()) +
               r_10_11_12_x() * (X.y() - ty()) +
               r_20_21_22_x() * (X.z() - tz());
    X.w() = 1.0f;
  }
  inline void ApplyInversely(const Point3D &X, Point3D &TIX) const {
    TIX.xyzw() = r_00_01_02_x() * (X.x() - tx()) +
                 r_10_11_12_x() * (X.y() - ty()) +
                 r_20_21_22_x() * (X.z() - tz());
    TIX.w() = 1.0f;
  }
  inline Point3D GetAppliedInversely(const Point3D &X) const { Point3D TIX; ApplyInversely(X, TIX); return TIX; }
  inline void ApplyInversely(const Point2D &x, const float z, Point3D &X) const {
    X.xyzw() = r_00_01_02_x() * (z * x.x() - tx()) +
               r_10_11_12_x() * (z * x.y() - ty()) +
               r_20_21_22_x() * (z - tz());
    X.w() = 1;
  }
  inline Point3D GetAppliedInversely(const Point2D &x, const float z) const {
    Point3D X;
    ApplyInversely(x, z, X);
    return X;
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e\n", r00(), r01(), r02(), tx());
      UT::Print("%e %e %e %e\n", r10(), r11(), r12(), ty());
      UT::Print("%e %e %e %e\n", r20(), r21(), r22(), tz());
    } else {
      UT::Print("%f %f %f %f\n", r00(), r01(), r02(), tx());
      UT::Print("%f %f %f %f\n", r10(), r11(), r12(), ty());
      UT::Print("%f %f %f %f\n", r20(), r21(), r22(), tz());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    const std::string _str(str.size(), ' ');
    if (e) {
      UT::Print("%s%e %e %e %e\n",  str.c_str(), r00(), r01(), r02(), tx());
      UT::Print("%s%e %e %e %e\n", _str.c_str(), r10(), r11(), r12(), ty());
      UT::Print("%s%e %e %e %e\n", _str.c_str(), r20(), r21(), r22(), tz());
    } else {
      UT::Print("%s%f %f %f %f\n",  str.c_str(), r00(), r01(), r02(), tx());
      UT::Print("%s%f %f %f %f\n", _str.c_str(), r10(), r11(), r12(), ty());
      UT::Print("%s%f %f %f %f\n", _str.c_str(), r20(), r21(), r22(), tz());
    }
  }

  inline bool Load(const std::string fileName) {
    FILE *fp = fopen(fileName.c_str(), "r");
    return fp && UT::Load<float>(&r00(), 12, fp);
  }

  inline void Random(const float thMax, const float tMax) {
    Rotation3D::Random(thMax);
    tx() = UT::Random<float>(-tMax, tMax);
    ty() = UT::Random<float>(-tMax, tMax);
    tz() = UT::Random<float>(-tMax, tMax);
  }
  static inline Rigid3D GetRandom(const float thMax, const float tMax) {
    Rigid3D T;
    T.Random(thMax, tMax);
    return T;
  }

  inline bool AssertEqual(const Rigid3D &T, const int verbose = 1, const std::string str = "",
                          const float rEps = 0.001745329252f, const float tEps = 0.0f) const {
    //const LA::AlignedVector3f dt =  - T.GetTranslation();
    if (Rotation3D::AssertEqual(T, verbose, str + ".m_R", rEps) &&
        GetTranslation().AssertEqual(T.GetTranslation(), verbose, str + ".m_t", tEps)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      T.Print(verbose > 1);
    }
    return false;
  }

  static inline void ABI(const Rigid3D &Ta, const Rigid3D &Tb, Rigid3D &TabI) {
    LA::AlignedMatrix3x3f::ABT(Ta, Tb, TabI);
    TabI.MakeOrthogonal();
    TabI.SetTranslation(Ta.GetTranslation() - TabI.GetAppliedRotation(Tb.GetTranslation()));
  }
  static inline void ABI(const Row &Ta, const Rigid3D &Tb, Row &TabI) {
    LA::AlignedMatrix3x3f::aBT(Ta, Tb, TabI);
    TabI.t() = Ta.t() - (TabI.r0_r1_r2_t() * Tb.GetTranslation().v012r()).vsum_012();
  }
};
#endif  // _RIGID_H_
