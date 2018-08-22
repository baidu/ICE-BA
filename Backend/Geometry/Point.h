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
#ifndef _POINT_H_
#define _POINT_H_

#include "Vector3.h"
#include "Matrix2x3.h"

class Point2D : public LA::Vector2f {
 public:
  inline Point2D() : LA::Vector2f() {}
  inline Point2D(const float x, const float y) : LA::Vector2f(x, y) {}
  inline Point2D(const float *x) { memcpy(this, x, sizeof(Point2D)); }
  inline Point2D(const LA::Vector2f &x) { memcpy(this, &x, sizeof(Point2D)); }
};

class Point2DCovariance : public LA::SymmetricMatrix2x2f {

 public:

  inline Point2DCovariance() {}
  inline Point2DCovariance(const LA::SymmetricMatrix2x2f &S) { *this = S; }
  inline Point2DCovariance(const float sxx, const float sxy,
                           const float syy) : LA::SymmetricMatrix2x2f(sxx, sxy, syy) {}

  inline const float& sxx() const { return m00(); } inline float& sxx() { return m00(); }
  inline const float& sxy() const { return m01(); } inline float& sxy() { return m01(); }
  inline const float& syy() const { return m11(); } inline float& syy() { return m11(); }

  inline void operator = (const LA::SymmetricMatrix2x2f &M) {
    memcpy(m_data, &M, sizeof(Point2DCovariance));
  }
  inline void operator += (const LA::Vector2f &x) {
    sxx() += x.x() * x.x();
    sxy() += x.x() * x.y();
    syy() += x.y() * x.y();
  }
  inline void operator += (const Point2DCovariance &S) {
    sxx() += S.sxx();
    sxy() += S.sxy();
    syy() += S.syy();
  }

  inline float Area() const {
    return UT_PI * sqrtf(Determinant());
  }

  enum EigenDecomposeResult {
    EIGEN_DECOMPOSE_SUCCESS_X_SUCCESS_Y,
    EIGEN_DECOMPOSE_SUCCESS_X_FAIL_Y,
    EIGEN_DECOMPOSE_FAIL_X_SUCCESS_Y,
    EIGEN_DECOMPOSE_FAIL_X_FAIL_Y
  };
  inline EigenDecomposeResult EigenDecompose(LA::AlignedMatrix2x2f &U, LA::Vector2f &l,
                                             const float eps) const {
    l.x() = (sxx() - syy()) * 0.5f;
    const float r = sqrtf(l.x() * l.x() + sxy() * sxy());
    l.y() = l.x() + syy();
    l.x() = l.y() + r;
    l.y() = l.y() - r;
    U.m00() = -sxy();
    U.m10() = sxx() - l.x();
    U.m01() = -sxy();
    U.m11() = sxx() - l.y();
    const xp128f lv2 = U.m_00_01_10_11() * U.m_00_01_10_11();

    const float lv0 = sqrtf(lv2[0] + lv2[2]);

    const bool sccX = lv0 > eps;
    if (sccX) {
      const float sv0 = 1.0f / lv0;
      U.m00() *= sv0;
      U.m10() *= sv0;
    } else {
      l.x() = 0.0f;
      U.m00() = U.m10() = 0.0f;
    }
    const float lv1 = sqrtf(lv2[1] + lv2[3]);

    const bool sccY = lv1 > eps;
    if (sccY) {
      const float sv1 = 1 / lv1;
      U.m01() *= sv1;
      U.m11() *= sv1;
    } else {
      l.y() = 0.0f;
      U.m01() = U.m11() = 0.0f;
    }
    if (sccX && sccY) {
      return EIGEN_DECOMPOSE_SUCCESS_X_SUCCESS_Y;
    } else if (sccX && !sccY) {
      return EIGEN_DECOMPOSE_SUCCESS_X_FAIL_Y;
    } else if (!sccX && sccY) {
      return EIGEN_DECOMPOSE_FAIL_X_SUCCESS_Y;
    } else {
      return EIGEN_DECOMPOSE_FAIL_X_FAIL_Y;
    }
  }
  inline void EigenCompose(const LA::AlignedMatrix2x2f &U, const LA::Vector2f &l) {
    const xp128f t = U.m_00_01_10_11() * U.m_00_01_10_11();
    sxx() = l.x() * t[0] + l.y() * t[1];
    sxy() = l.x() * U.m00() * U.m10() + l.y() * U.m01() * U.m11();
    syy() = l.x() * t[2] + l.y() * t[3];
  }
};

class Point3D : public LA::AlignedVector3f {
 public:
  inline Point3D() { w() = 1.0f; }
  inline Point3D(const float x, const float y, const float z) { Set(x, y, z); }
  inline Point3D(const float *X) { Set(X); }
  inline Point3D(const LA::Vector2f &x, const float z) { Set(x, z); }
  inline Point3D(const AlignedVector3f &X) { Set(X.xyzr()); }
  inline Point3D(const xp128f &X) { Set(X); }

  inline const xp128f& xyzw() const { return xyzr(); }  inline xp128f& xyzw() { return xyzr(); }
  inline const float& w() const { return r (); }      inline float& w() { return r (); }

  inline void operator = (const Point3D &X) { xyzw() = X.xyzw(); }
  inline void operator = (const AlignedVector3f &X) { xyzw() = X.xyzr(); w() = 1.0f; }
  inline void operator += (const AlignedVector3f &X) {
    xyzw() += X.xyzr(); w() = 1.0f;
  }
  inline Point3D operator + (const AlignedVector3f &X) const {
    return Point3D(xyzw() + X.xyzr());
  }
  inline Point3D operator - (const AlignedVector3f &X) const {
    return Point3D(xyzw() - X.xyzr());
  }
  inline Point3D operator * (const float s) const {
    return Point3D(xyzw() * s);
  }
  inline Point3D operator * (const xp128f &s) const { return Point3D(xyzw() * s); }
  inline Point3D operator / (const float d) const { return Point3D(xyzw() * (1.0f / d)); }

  inline void Set(const float *X) { memcpy(this, X, 12); w() = 1.0f; }
  inline void Set(const double *X) { v012r().vset_all_lane(float(X[0]), float(X[1]), float(X[2]), 1.0f); }
  inline void Set(const float x, const float y, const float z) { xyzw().vset_all_lane(x, y, z, 1.0f); }
  inline void Set(const LA::Vector2f &x, const float z) { xyzw().vset_all_lane(x.x() * z, x.y() * z, z, 1.0f); }
  inline void Set(const xp128f &X) { xyzw() = X; w() = 1.0f; }

  inline void MakeZero() { memset(this, 0, 12); w() = 1.0f; }

  inline void Normalize() {
    xyzw() *= 1.0f / sqrtf(SquaredLength());
    w() = 1.0f;
  }
  inline Point3D Cross(const AlignedVector3f &v) const {
    return Point3D(LA::AlignedVector3f::Cross(v));
  }

  inline void Load(FILE *fp) { LA::AlignedVector3f::Load(fp); w() = 1.0f; }

  static inline void apb(const AlignedVector3f &a, const Point3D &Xb, Point3D &Xapb) {
    Xapb.xyzw() = a.xyzr() + Xb.xyzw();
    Xapb.w() = 1.0f;
  }

  inline void Random(const float pMax) {
    LA::AlignedVector3f::Random(pMax); w() = 1.0f;
  }
  static inline Point3D GetRandom(const float pMax) {
    Point3D p; p.Random(pMax); return p;
  }
};

#ifdef CFG_DEBUG_EIGEN
class EigenPoint2D : public EigenVector2f {
 public:
  inline EigenPoint2D() : EigenVector2f() {}
  inline EigenPoint2D(const EigenVector2f &e_x) : EigenVector2f(e_x) {}
  inline EigenPoint2D(const float x, const float y) : EigenVector2f(x, y) {}
};
class EigenPoint3D : public EigenVector3f {
 public:
  inline EigenPoint3D() : EigenVector3f() {}
  inline EigenPoint3D(const EigenVector3f &e_X) : EigenVector3f(e_X) {}
  inline EigenPoint3D(const EigenPoint2D &e_x) : EigenVector3f(e_x.x(), e_x.y(), 1.0f) {}
  inline EigenPoint3D(const EigenPoint2D &e_x, const float z) : EigenVector3f(e_x.x() * z,
                                                                                e_x.y() * z, z) {}
  inline EigenPoint3D(const float x, const float y, const float z) : EigenVector3f(x, y, z) {}
  inline EigenMatrix2x3f GetJacobianProjection() const {
    EigenMatrix2x3f e_J;
    const Eigen::Vector3f &e_X = *this;
    const float zI = 1.0f / e_X.z(), z2I = zI * zI;
    e_J(0, 0) = zI;   e_J(0, 1) = 0.0f; e_J(0, 2) = -e_X.x() * z2I;
    e_J(1, 0) = 0.0f; e_J(1, 1) = zI;   e_J(1, 2) = -e_X.y() * z2I;
    return e_J;
  }
};
#endif
#endif
