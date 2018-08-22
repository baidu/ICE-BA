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
#ifndef _INTRINSIC_H_
#define _INTRINSIC_H_

#include "Rotation.h"
#include "Table.h"
#include "AlignedVector.h"

class Intrinsic {
 public:

  class Parameter {
   public:
    inline void Set(const float fx, const float fy, const float cx, const float cy) {
      m_fx = fx;
      m_fy = fy;
      m_cx = cx;
      m_cy = cy;
      m_fxI = 1.0f / fx;
      m_fyI = 1.0f / fy;
      m_fxx = fx * fx;
      m_fxy = fx * fy;
      m_fyy = fy * fy;
      m_fxxI = 1.0f / m_fxx;
      m_fxyI = 1.0f / m_fxy;
      m_fyyI = 1.0f / m_fyy;
    }
    inline void ImageToNormalized(Point2D &x) const {
      x.x() = (x.x() - m_cx) * m_fxI;
      x.y() = (x.y() - m_cy) * m_fyI;
    }
    inline void ImageToNormalized(const float *x, Point2D &xn) const {
      xn.x() = (x[0] - m_cx) * m_fxI;
      xn.y() = (x[1] - m_cy) * m_fyI;
    }
    inline Point2D GetImageToNormalized(const Point2D &x) const {
      Point2D xn;
      ImageToNormalized(x, xn);
      return xn;
    }
    template<typename TYPE> inline void ImageToNormalized(const TYPE xx, const TYPE xy, 
                                                          Point2D &xn) const {
        xn.x() = (xx - m_cx) * m_fxI;
        xn.y() = (xy - m_cy) * m_fyI;
    }
    template<typename TYPE> inline Point2D GetImageToNormalized(const TYPE xx, const TYPE xy) const {
      Point2D xn;
      ImageToNormalized<TYPE>(xx, xy, xn);
      return xn;
    }
    template<typename TYPE> inline void ImageToNormalized(const TYPE xx, const TYPE xy, 
                                                          float &xn, float &yn) const {
        xn = (xx - m_cx) * m_fxI;
        yn = (xy - m_cy) * m_fyI;
    }
    inline void ImageToNormalized(const Point2DCovariance &S, Point2DCovariance &Sn) const {
      Sn.sxx() = S.sxx() * m_fxxI;
      Sn.sxy() = S.sxy() * m_fxyI;
      Sn.syy() = S.syy() * m_fyyI;
    }
    inline void ImageToNormalized(const float S[2][2], Point2DCovariance &Sn) const {
      Sn.sxx() = S[0][0] * m_fxxI;
      Sn.sxy() = S[0][1] * m_fxyI;
      Sn.syy() = S[1][1] * m_fyyI;
    }
    inline void NormalizedToImage(Point2D &x) const {
      x.x() = m_fx * x.x() + m_cx;
      x.y() = m_fy * x.y() + m_cy;
    }
    inline void NormalizedToImage(const Point2D &xn, Point2D &x) const {
      x.x() = m_fx * xn.x() + m_cx;
      x.y() = m_fy * xn.y() + m_cy;
    }
    inline Point2D GetNormalizedToImage(const Point2D &xn) const {
      Point2D x;
      NormalizedToImage(xn, x);
      return x;
    }
    inline void DownSample() {
      m_fx *= 0.5f;
      m_fy *= 0.5f;
      m_cx *= 0.5f;
      m_cy *= 0.5f;
    }
    inline void GetDownSampled(Parameter &k) const {
      k.m_fx = m_fx * 0.5f;
      k.m_fy = m_fy * 0.5f;
      k.m_cx = m_cx * 0.5f;
      k.m_cy = m_cy * 0.5f;
      memcpy(k.m_ds, m_ds, sizeof(m_ds));
      memcpy(k.m_jds, m_jds, sizeof(m_jds));
    }
    inline void AssertConsistency(const bool fishEye, const bool radial6, const bool tangential) const {
      UT_ASSERT(m_fxI == 1.0f / m_fx && m_fyI == 1.0f / m_fy);
      UT_ASSERT(m_fxx == m_fx * m_fx && m_fxy == m_fx * m_fy && m_fyy == m_fy * m_fy);
      UT_ASSERT(m_fxxI == 1.0f / m_fxx && m_fxyI == 1.0f / m_fxy && m_fyyI == 1.0f / m_fyy);
      if (fishEye) {
      } else {
        UT_ASSERT(m_jds[1] == m_ds[1] * 2.0f);
        UT_ASSERT(m_jds[4] == m_ds[4] * 3.0f);
        if (radial6) {
          UT_ASSERT(m_jds[6] == m_ds[6] * 2.0f);
          UT_ASSERT(m_jds[7] == m_ds[7] * 3.0f);
        }
        if (tangential) {
          UT_ASSERT(m_jds[2] == m_ds[2] * 2.0f);
          UT_ASSERT(m_jds[3] == m_ds[3] * 2.0f);
        }
      }
    }
   public:
    float m_fx, m_fy, m_cx, m_cy;
    float m_fxI, m_fyI;
    float m_fxx, m_fxy, m_fyy;
    float m_fxxI, m_fxyI, m_fyyI;
    float m_ds[8], m_jds[8];
  };

  class RectificationMap {
   public:
    inline bool Empty() const { return m_xs.Empty(); }
    inline void Resize(const int N) { m_xs.Resize(N); m_ws.Resize(N); }
    inline void Set(RectificationMap &RM) {
      m_xs.Set(RM.m_xs.Data(), RM.m_xs.Size());
      m_ws.Set(RM.m_ws.Data(), RM.m_ws.Size());
    }
    inline void Bind(RectificationMap &RM) {
      m_xs.Bind(RM.m_xs.Data(), RM.m_xs.Size());
      m_ws.Bind(RM.m_ws.Data(), RM.m_ws.Size());
    }
    inline void Set(const Intrinsic &K
#ifdef CFG_STEREO
                  , const Rotation3D *Rr = NULL
#endif
                  ) {
      struct T {
        float x, x2;
#ifdef CFG_STEREO
        float rx, ry, rz;
#endif
      };
      const int w = K.w(), h = K.h();
      std::vector<T> ts(w);
      for (int xr = 0; xr < w; ++xr) {
        T &t = ts[xr];
        t.x = (xr - K.cx()) * K.fxI();
#ifdef CFG_STEREO
        if (Rr) {
          t.rx = Rr->r00() * t.x;
          t.ry = Rr->r01() * t.x;
          t.rz = Rr->r02() * t.x;
        }
        else
#endif
        {
          t.x2 = t.x * t.x;
        }
      }
      Point2D xd;
      float x, x2, y, y2, d2y, d3y, dr;
#ifdef CFG_STEREO
      float rx, ry, rz;
#endif
      const int N = w * h;
      Resize(N);
      const Parameter &k = K.k();
      const bool fishEye = K.FishEye(), radial6 = K.Radial6(), tangential = K.Tangential();
      const int ixMax = w - 2, iyMax = h - 2;
      for (int yr = 0, i = 0; yr < h; ++yr) {
        const float _y = (yr - K.cy()) * K.fyI();
#ifdef CFG_STEREO
        if (Rr) {
          rx = Rr->r10() * _y + Rr->r20();
          ry = Rr->r11() * _y + Rr->r21();
          rz = Rr->r12() * _y + Rr->r22();
        } else
#endif
        {
          y = _y;
          y2 = y * y;
          if (tangential) {
            d2y = k.m_ds[2] * (y + y);
            d3y = k.m_ds[3] * (y + y);
          }
        }
        for (int xr = 0; xr < w; ++xr, ++i) {
          const T &t = ts[xr];
#ifdef CFG_STEREO
          if (Rr) {
            const float d = 1.0f / (t.rz + rz);
            x = (t.rx + rx) * d;
            y = (t.ry + ry) * d;
            x2 = x * x;
            y2 = y * y;
            if (tangential) {
              d2y = k.m_ds[2] * (y + y);
              d3y = k.m_ds[3] * (y + y);
            }
          } else
#endif
          {
            x = t.x;
            x2 = t.x2;
          }
          const float r2 = x2 + y2;
          if (fishEye) {
            const float r = sqrtf(r2), t1 = UT_ATANF(r), t2 = t1 * t1;
            const float t4 = t2 * t2, t6 = t2 * t4, t8 = t4 * t4;
            const float s = t1 * (k.m_ds[3] * t8 + k.m_ds[2] * t6 + k.m_ds[1] * t4 +
                                  k.m_ds[0] * t2 + 1.0f) / r;
            xd.x() = s * x;
            xd.y() = s * y;
          } else {
            const float r4 = r2 * r2, r6 = r2 * r4;
            dr = k.m_ds[4] * r6 + k.m_ds[1] * r4 + k.m_ds[0] * r2 + 1.0f;
            if (radial6)
              dr /= k.m_ds[7] * r6 + k.m_ds[6] * r4 + k.m_ds[5] * r2 + 1.0f;
            xd.x() = dr * x;
            xd.y() = dr * y;
            if (tangential) {
              const float dx = d2y * x + k.m_ds[3] * (r2 + x2 + x2);
              const float dy = k.m_ds[2] * (r2 + y2 + y2) + d3y * t.x;
              xd.x() = dx + xd.x();
              xd.y() = dy + xd.y();
            }
          }
          xd.x() = k.m_fx * xd.x() + k.m_cx;
          xd.y() = k.m_fy * xd.y() + k.m_cy;
          const int ix = int(xd.x()), iy = int(xd.y());
          if (ix < 0 || iy < 0 || ix > ixMax || iy > iyMax) {
            m_xs[i].Set(-1, -1);
          } else {
            m_xs[i].Set(ix, iy);
            UT::ImageInterpolateWeight(xd.x() - ix, xd.y() - iy, m_ws[i]);
          }
        }
      }
    }
   public:
    AlignedVector<LA::Vector2i> m_xs;
    AlignedVector<xp128f> m_ws;
  };

 public:

  inline const int& w() const { return m_w; }
  inline const int& h() const { return m_h; }
  inline const Parameter& k() const { return m_k; }
  inline const bool NeedRectification() const { return m_needRect == 1; }
  inline const bool NeedUndistortion() const { return m_needUndist == 1; }
  inline const bool FishEye() const { return m_fishEye == 1; }
  inline const bool Radial6() const { return m_radial6 == 1; }
  inline const bool Tangential() const { return m_tangential == 1; }

  // [NOTE] GCC extension treats xp128f as __vector(4) float and allows direct indexing
  inline const float& fx () const { return m_fx[0]; }
  inline const float& fy () const { return m_fy[0]; }
  inline const float& fxI() const { return m_fxI[0]; }
  inline const float& fyI() const { return m_fyI[0]; }
  inline const float& cx () const { return m_cx[0]; }
  inline const float& cy () const { return m_cy[0]; }
  inline const float& fxIcx() const { return m_fxIcx[0]; }
  inline const float& fyIcy() const { return m_fyIcy[0]; }

  inline const float& fxx () const { return m_fxx; }
  inline const float& fxxI() const { return m_fxxI; }
  inline const float& fxy () const { return m_fxy; }
  inline const float& fxyI() const { return m_fxyI; }
  inline const float& fyy () const { return m_fyy; }
  inline const float& fyyI() const { return m_fyyI; }

  inline const xp128f& Fx () const { return m_fx; }
  inline const xp128f& Fy () const { return m_fy; }
  inline const xp128f& FxI() const { return m_fxI; }
  inline const xp128f& FyI() const { return m_fyI; }
  inline const xp128f& Cx () const { return m_cx; }
  inline const xp128f& Cy () const { return m_cy; }
  inline const xp128f& FxIcx() const { return m_fxIcx; }
  inline const xp128f& FyIcy() const { return m_fyIcy; }

  inline LA::AlignedMatrix3x3f operator * (const LA::AlignedMatrix3x3f &M) const {
    LA::AlignedMatrix3x3f KM;
    KM.m_00_01_02_r0() = Fx() * M.m_00_01_02_r0() + Cx() * M.m_20_21_22_r2();
    KM.m_10_11_12_r1() = Fy() * M.m_10_11_12_r1() + Cy() *  M.m_20_21_22_r2();
    KM.m_20_21_22_r2() = M.m_20_21_22_r2();
    return KM;
  }
  inline LA::AlignedVector3f operator * (const LA::AlignedVector3f &v) const {
    LA::AlignedVector3f Kv;
    Kv.x() = fx() * v.x() + cx() * v.z();
    Kv.y() = fy() * v.y() + cy() * v.z();
    Kv.z() = v.z();
    return Kv;
  }

  inline void Set(const int w, const int h, const float fx, const float fy = 0.0f,
                  const float cx = 0.0f, const float cy = 0.0f, const float *ds = NULL,
                  const bool fishEye = false, const float fxr = 0.0f, const float fyr = 0.0f,
                  const float cxr = 0.0f, const float cyr = 0.0f, const Rotation3D *Rr = NULL) {
    m_w = w;
    m_h = h;
    const float _cxr = cxr == 0.0f ? (w - 1) * 0.5f : cxr;
    const float _cyr = cyr == 0.0f ? (h - 1) * 0.5f : cyr;
    m_k.Set(fx, fy == 0.0f ? fx : fy, cx == 0.0f ? _cxr : cx, cy == 0.0f ? _cyr : cy);
    m_radial6 = ds && (ds[5] != 0.0f || ds[6] != 0.0f || ds[7] != 0.0f) ? 1 : 0;
    m_tangential = ds && (ds[2] != 0.0f || ds[3] != 0.0f) ? 1 : 0;
    m_needUndist = m_radial6 != 0 || m_tangential != 0 ||
                  (ds && (ds[0] != 0.0f || ds[1] != 0.0f || ds[4] != 0.0f)) ? 1 : 0;
    if (m_needUndist || fxr != 0.0f || fyr != 0.0f || Rr) {
      m_needRect = 1;
      if (m_needUndist != 0) {
        m_fishEye = fishEye ? 1 : 0;
        memcpy(m_k.m_ds, ds, sizeof(m_k.m_ds));
        memset(m_k.m_jds, 0, sizeof(m_k.m_jds));
        if (fishEye) {
          m_radial6 = 0;
          m_tangential = 0;
        } else {
          m_k.m_jds[1] = ds[1] * 2.0f;
          m_k.m_jds[4] = ds[4] * 3.0f;
          if (m_radial6) {
            m_k.m_jds[6] = ds[6] * 2.0f;
            m_k.m_jds[7] = ds[7] * 3.0f;
          }
          if (m_tangential) {
            m_k.m_jds[2] = ds[2] * 2.0f;
            m_k.m_jds[3] = ds[3] * 2.0f;
          }
        }
      } else {
        m_fishEye = 0;
        memset(m_k.m_ds, 0, sizeof(m_k.m_ds));
        memset(m_k.m_jds, 0, sizeof(m_k.m_jds));
      }
      //Set(m_k.m_fx, m_k.m_fy, m_k.m_cx, m_k.m_cy);
      Set(m_k.m_fx, m_k.m_fy, _cxr, _cyr);
      const float _fxr = fxr == 0.0f ? GetRectifiedFocal() : fxr;
      const float _fyr = fyr == 0.0f ? _fxr : fyr;
      //Set(_fxr, _fyr, m_k.m_cx, m_k.m_cy);
      Set(_fxr, _fyr, _cxr, _cyr);
    } else {
      Set(m_k.m_fx, m_k.m_fy, m_k.m_cx, m_k.m_cy);
      m_needRect = 0;
      m_fishEye = 0;
    }
  }
  inline void DownSample() {
    m_w /= 2;
    m_h /= 2;
    m_k.DownSample();
    Set(fx() * 0.5f, fy() * 0.5f, cx() * 0.5f, cy() * 0.5f);
  }
  inline void GetDownSampled(Intrinsic &K) const {
    K.m_w = m_w / 2;
    K.m_w = SIMD_BYTE_FLOOR(K.m_w);
    K.m_h = m_h / 2;
    K.m_flag[0] = m_flag[0];
    K.m_flag[1] = m_flag[1];
    m_k.GetDownSampled(K.m_k);
    K.Set(fx() * 0.5f, fy() * 0.5f, cx() * 0.5f, cy() * 0.5f);
  }
  inline double GetFovX() const { return FocalToFov(m_w, fxI()); }
  inline double GetFovY() const { return FocalToFov(m_h, fyI()); }
  static inline double FocalToFov(const int r, const float fI) {
    return UT_ATANF(r * 0.5 * fI) * 2 * UT_FACTOR_RAD_TO_DEG;
  }
  static inline float FovToFocal(const int r, const double fov) {
    return float(r / (2 * UT_TANF(fov * 0.5 * UT_FACTOR_DEG_TO_RAD)));
  }

  inline void Get(LA::AlignedMatrix3x3f &K) const {
    K.m00() = fx();       K.m01() = 0.0f;         K.m02() = cx();
    K.m10() = 0.0f;       K.m11() = fy();         K.m12() = cy();
    K.m20() = 0.0f;       K.m21() = 0.0f;         K.m22() = 1.0f;
  }
  inline void GetInverse(LA::AlignedMatrix3x3f &KI) const {
    KI.m00() = fxI();     KI.m01() = 0.0f;      KI.m02() = -cx() * fxI();
    KI.m10() = 0.0f;      KI.m11() = fyI();     KI.m12() = -cy() * fyI();
    KI.m20() = 0.0f;      KI.m21() = 0.0f;      KI.m22() = 1.0f;
  }
  inline void GetTranspose(LA::AlignedMatrix3x3f &KT) const {
    KT.m00() = fx();      KT.m01() = 0.0f;      KT.m02() = 0.0f;
    KT.m10() = 0.0f;      KT.m11() = fy();      KT.m12() = 0.0f;
    KT.m20() = cx();      KT.m21() = cy();      KT.m22() = 1.0f;
  }
  inline void GetTransposeInverse(LA::AlignedMatrix3x3f &KTI) const {
    KTI.m00() = fxI();      KTI.m01() = 0.0f;     KTI.m02() = 0.0f;
    KTI.m10() = 0.0f;     KTI.m11() = fyI();      KTI.m12() = 0.0f;
    KTI.m20() = -cx() * fxI();  KTI.m21() = -cy() * fyI();  KTI.m22() = 1.0f;
  }
  inline void GetInverseTranspose(LA::AlignedMatrix3x3f &KIT) const { GetTransposeInverse(KIT); }
  inline void GetInverse(Intrinsic &KI) const { KI.Set(fxI(), fyI(), -cx() * fxI(), -cy() * fyI()); }

  inline float GetRectifiedFocal() const {
    const float xdMax = m_k.m_cx / m_k.m_fx;
    const float ydMax = m_k.m_cy / m_k.m_fy;
    Point2D xd, xn;

#if 0
//#if 1
    UT::DebugStart();
#endif
    float xnMin = -xdMax, xnMax = xdMax;
    float ynMin = -ydMax, ynMax = ydMax;
    for (int i = 0; i < 4; ++i) {
      switch (i) {
      case 0: xd.Set( xdMax,  ydMax); break;
      case 1: xd.Set( xdMax, -ydMax); break;
      case 2: xd.Set(-xdMax,  ydMax); break;
      case 3: xd.Set(-xdMax, -ydMax); break;
      }
      if (!Undistort(xd, &xn)) {
        continue;
      }
#if 0
//#if 1
      xd.Print("xd = ", true, false);
      xn.Print(" xn = ", true, true);
#endif
      xnMin = std::min(xnMin, xn.x());
      xnMax = std::max(xnMax, xn.x());
      ynMin = std::min(ynMin, xn.y());
      ynMax = std::max(ynMax, xn.y());
    }
    const float fxr = m_k.m_fx / (xnMax - xnMin) * (xdMax + xdMax);
    const float fyr = m_k.m_fy / (ynMax - ynMin) * (ydMax + ydMax);
#if 0
//#if 1
    UT::Print("%e %e\n", fxr, fyr);
#endif
#if 0
//#if 1
    UT::DebugStop();
#endif
    return std::min(fxr, fyr);
  }

  inline void NormalizedToImage(Point2D &x) const {
    x.x() = fx() * x.x() + cx();
    x.y() = fy() * x.y() + cy();
  }
  inline void NormalizedToImage(const Point2D &xn, Point2D &x) const {
    x.x() = fx() * xn.x() + cx();
    x.y() = fy() * xn.y() + cy();
  }
  inline Point2D GetNormalizedToImage(const Point2D &xn) const {
    Point2D x;
    NormalizedToImage(xn, x);
    return x;
  }
  inline void NormalizedToImage2(const xp128f &xn, xp128f &x) const {
    x = m_f * xn + m_c;
  }
  inline void NormalizedToImage2(xp128f &x) const { x = m_f * x +  m_c; }
  inline void NormalizedToImageN(AlignedVector<Point2D> &xs) const {
    const int N = int(xs.Size()), NF = N - (N & 1);
    xp128f *x = (xp128f *) xs.Data();
    for (int i = 0; i < NF; i += 2, ++x) {
      NormalizedToImage2(*x);
    }
    if (NF != N) {
      NormalizedToImage(xs[NF]);
    }
  }
  inline void NormalizedToImageN(const AlignedVector<Point2D> &xns,
                                 AlignedVector<Point2D> &xs) const {
    const int N = int(xs.Size()), NF = N - (N & 1);
    xs.Resize(N);
    const xp128f *xn = (const xp128f *) xns.Data();
    xp128f *x = (xp128f *) xs.Data();
    for (int i = 0; i < NF; i += 2, ++x, ++x) {
      NormalizedToImage2(*xn, *x);
    }
    if (NF != N) {
      NormalizedToImage(xns[NF], xs[NF]);
    }
  }
  inline void NormalizedToImageN(const Point2D *xns, const int N, Point2D *xs) const {
    const int NF = N - (N & 1);
    const xp128f *xn = (const xp128f *) xns;
    xp128f *x = (xp128f *) xs;
    for (int i = 0; i < NF; i += 2, ++xn, ++x) {
      NormalizedToImage2(*xn, *x);
    }
    if (NF != N) {
      NormalizedToImage(xns[NF], xs[NF]);
    }
  }
  inline void NormalizedToImageN(Point2D *xs, const int N) const {
    const int NF = N - (N & 1);
    xp128f *x = (xp128f *) xs;
    for (int i = 0; i < NF; i += 2, ++x) {
      NormalizedToImage2(*x);
    }
    if (NF != N) {
      NormalizedToImage(xs[NF]);
    }
  }
  //inline void NormalizedToImage(Line2D &l) const
  //{
  //  l.c() -= l.a() * cxfxI() + l.b() * cyfyI();
  //  l.a() *= fxI();
  //  l.b() *= fyI();
  //  l.Normalize();
  //}
  //inline void NormalizedToImage(const Line2D &ln, Line2D &Kl) const
  //{
  //  Kl.c() = ln.c() - (ln.a() * cxfxI() + ln.b() * cyfyI());
  //  Kl.a() = ln.a() * fxI();
  //  Kl.b() = ln.b() * fyI();
  //  Kl.Normalize();
  //}
  inline void NormalizedToImage(LA::SymmetricMatrix2x2f &S) const {
    S.m00() *= fxx();
    S.m01() *= fxy();
    S.m11() *= fyy();
  }
  inline void NormalizedToImage(const LA::SymmetricMatrix2x2f &Sn, LA::SymmetricMatrix2x2f &S) const {
    S.m00() = Sn.m00() * fxx();
    S.m01() = Sn.m01() * fxy();
    S.m11() = Sn.m11() * fyy();
  }
  inline LA::SymmetricMatrix2x2f GetNormalizedToImage(const LA::SymmetricMatrix2x2f &Sn) const {
    Point2DCovariance S; NormalizedToImage(Sn, S); return S;
  }
  inline void NormalizedToImage(const LA::AlignedMatrix3x3f &Hn, LA::AlignedMatrix3x3f &H) const {
    NormalizedToImage(*this, *this, Hn, H);
  }
  inline LA::AlignedMatrix3x3f GetNormalizedToImage(const LA::AlignedMatrix3x3f &Hn) const {
    LA::AlignedMatrix3x3f H; NormalizedToImage(Hn, H); return H;
  }
  // H = K2 * Hn * K1^{-1}
  static inline void NormalizedToImage(const Intrinsic &K1, const Intrinsic &K2,
                                       const LA::AlignedMatrix3x3f &Hn, LA::AlignedMatrix3x3f &H) {
    LA::AlignedMatrix3x3f M;
    M.m_00_01_02_r0() = K2.Fx() * Hn.m_00_01_02_r0() + K2.Cx() * Hn.m_20_21_22_r2();
    M.m_10_11_12_r1() = K2.Fy() * Hn.m_10_11_12_r1() + K2.Cy() * Hn.m_20_21_22_r2();
    M.m_20_21_22_r2() = Hn.m_20_21_22_r2();
    M.Transpose();
    H.m_00_01_02_r0() = M.m_00_01_02_r0() * K1.FxI();
    H.m_10_11_12_r1() = M.m_10_11_12_r1() * K1.FyI();
    H.m_20_21_22_r2() = M.m_20_21_22_r2() - (M.m_00_01_02_r0() * K1.FxIcx() + M.m_10_11_12_r1() * K1.FyIcy());
    H.Transpose();
  }

  inline void ImageToNormalized(Point2D &x) const {
    x.x() = (x.x() - cx()) * fxI();
    x.y() = (x.y() - cy()) * fyI();
  }
  inline void ImageToNormalized(const Point2D &x, Point2D &xn) const {
    xn.x() = (x.x() - cx()) * fxI();
    xn.y() = (x.y() - cy()) * fyI();
  }
  inline Point2D GetImageToNormalized(const Point2D &x) const {
    Point2D xn;
    ImageToNormalized(x, xn);
    return xn;
  }
  template<typename TYPE> inline void ImageToNormalized(const TYPE xx, const TYPE xy,
                                                        Point2D &xn) const {
    xn.x() = (xx - cx()) * fxI();
    xn.y() = (xy - cy()) * fyI();
  }
  template<typename TYPE> inline Point2D GetImageToNormalized(const TYPE xx, const TYPE xy) const {
    Point2D xn;
    ImageToNormalized<TYPE>(xx, xy, xn);
    return xn;
  }
  template<typename TYPE> inline void ImageToNormalized(const TYPE xx, const TYPE xy, 
                                                        float &xn, float &yn) const {
    xn = (xx - cx()) * fxI();
    yn = (xy - cy()) * fyI();
  }
  inline void ImageToNormalized2(xp128f &x) const {
    x = (x - m_c) * m_fI;
  }
  inline void ImageToNormalized2(const xp128f &x, xp128f &xn) const { xn = (x - m_c) * m_fI; }
  inline void ImageToNormalizedN(AlignedVector<Point2D> &xs) const {
    const int N = int(xs.Size()), NF = N - (N & 1);
    xp128f *x = (xp128f *) xs.Data();
    for (int i = 0; i < NF; i += 2, ++x) {
      ImageToNormalized2(*x);
    }
    if (NF != N) {
      ImageToNormalized(xs[NF]);
    }
  }
  inline void ImageToNormalizedN(const AlignedVector<Point2D> &xs,
                                 AlignedVector<Point2D> &xns) const {
    const int N = int(xs.Size()), NF = N - (N & 1);
    xns.Resize(N);
    const xp128f *x = (const xp128f *) xs.Data();
    xp128f *xn = (xp128f *) xns.Data();
    for (int i = 0; i < NF; i += 2, ++x, ++xn) {
      ImageToNormalized2(*x, *xn);
    }
    if (NF != N) {
      ImageToNormalized(xs[NF], xns[NF]);
    }
  }
  inline void ImageToNormalizedN(const Point2D *xs, const int N, Point2D *xns) const {
    const int NF = N - (N & 1);
    const xp128f *x = (const xp128f *) xs;
    xp128f *xn = (xp128f *) xns;
    for (int i = 0; i < NF; i += 2, ++x, ++xn) {
      ImageToNormalized2(*x, *xn);
    }
    if (NF != N) {
      ImageToNormalized(xs[NF], xns[NF]);
    }
  }
  inline void ImageToNormalizedN(Point2D *xs, const int N) const {
    const int NF = N - (N & 1);
    xp128f *x = (xp128f *) xs;
    for (int i = 0; i < NF; i += 2, ++x) {
      ImageToNormalized2(*x);
    }
    if (NF != N) {
      ImageToNormalized(xs[NF]);
    }
  }
  //inline void ImageToNormalized(Line2D &l) const
  //{
  //  l.c() += cx() * l.a() + cy() * l.b();
  //  l.a() *= fx();
  //  l.b() *= fy();
  //  l.Normalize();
  //}
  inline void ImageToNormalized(const Point2DCovariance &S, Point2DCovariance &Sn) const {
    Sn.sxx() = S.sxx() * fxxI();
    Sn.sxy() = S.sxy() * fxyI();
    Sn.syy() = S.syy() * fyyI();
  }
  inline Point2DCovariance GetImageToNormalized(const Point2DCovariance &S) const {
    Point2DCovariance Sn; ImageToNormalized(S, Sn); return Sn;
  }

  inline void ImageOriginCornerToCenter(Point2D &x) const {
    x.x() -= cx();
    x.y() -= cy();
  }
  inline void ImageOriginCornerToCenter2(xp128f &x) const { x -= m_c; }
  inline void ImageOriginCornerToCenterN(AlignedVector<Point2D> &xs) const {
    const int N = int(xs.Size()), NF = N - (N & 1);
    xp128f *x = (xp128f *) xs.Data();
    for (int i = 0; i < NF; i += 2, ++x)
      ImageOriginCornerToCenter2(*x);
    if (NF != N)
      ImageOriginCornerToCenter(xs[NF]);
  }

  inline void Apply(LA::AlignedVector3f &v) const {
    v.x() = fx() * v.x() + cx() * v.z();
    v.y() = fy() * v.y() + cy() * v.z();
  }

  inline void Load(FILE *fp) {
    float fx, fy, cx, cy;
    fscanf(fp, "%f %f %f %f", &fx, &fy, &cx, &cy);
    Set(fx, fy, cx, cy);
  }
  inline bool Load(const char *fileName) {
    FILE *fp = fopen(fileName, "r");
    if (!fp) {
      return false;
    }
    Load(fp);
    fclose(fp);
    UT::PrintLoaded(fileName);
    return true;
  }
  inline void Save(FILE *fp) const { fprintf(fp, "%f %f %f %f\n", fx(), fy(), cx(), cy()); }
  inline bool Save(const char *fileName) const {
    FILE *fp = fopen(fileName, "w");
    if (!fp) {
      return false;
    }
    Save(fp);
    fclose(fp);
    UT::PrintSaved(fileName);
    return true;
  }

  inline void Print() const { UT::Print("%f %f %f %f\n", fx(), fy(), cx(), cy()); }

  inline void AssertConsistency() const { m_k.AssertConsistency(FishEye(), Radial6(), Tangential()); }

  class UndistortionMap {
   public:
    void Set(const Intrinsic &K);
    inline const bool Empty() const { return m_xns.Empty(); }
    inline Point2D Get(const Point2D &xd) const {
      const float x = m_fx * xd.x() + m_cx;
      const float y = m_fy * xd.y() + m_cy;

      xp128f w;
      Point2D xn;
      const int ix1 = int(x), ix2 = ix1 + 1, iy1 = int(y), iy2 = iy1 + 1;
#ifdef CFG_DEBUG
      UT_ASSERT(ix1 >= 0 && ix2 < m_xns.w() && iy1 >= 0 && iy2 < m_xns.h());
#endif
      const Point2D &xn11 = m_xns[iy1][ix1], &xn12 = m_xns[iy2][ix1];
      const Point2D &xn21 = m_xns[iy1][ix2], &xn22 = m_xns[iy2][ix2];
      UT::ImageInterpolateWeight(x - ix1, y - iy1, w);
      UT::ImageInterpolate<float, float>(xn11.x(), xn12.x(), xn21.x(), xn22.x(), w, xn.x());
      UT::ImageInterpolate<float, float>(xn11.y(), xn12.y(), xn21.y(), xn22.y(), w, xn.y());
      return xn;
    }
  protected:
    float m_fx, m_fy, m_cx, m_cy;
    Table<Point2D> m_xns;
  };

  inline void Distort(const Point2D &xn, Point2D *xd) const {
    const float *ds = m_k.m_ds, *jds = m_k.m_jds;
    const float x = xn.x(), x2 = x * x, y = xn.y(), y2 = y * y, r2 = x2 + y2;
    if (m_fishEye) {
      const float r = sqrtf(r2), t1 = UT_ATANF(r), t2 = t1 * t1;
      const float t4 = t2 * t2, t6 = t2 * t4, t8 = t4 * t4;
      const float s = t1 * (ds[3] * t8 + ds[2] * t6
        + ds[1] * t4 + ds[0] * t2 + 1.0f) / r;
      xd->x() = s * x;
      xd->y() = s * y;
    } else {
      const float r4 = r2 * r2, r6 = r2 * r4;
      float dr = ds[4] * r6 + ds[1] * r4 + ds[0] * r2 + 1.0f;
      if (m_radial6)
        dr /= ds[7] * r6 + ds[6] * r4 + ds[5] * r2 + 1.0f;
      xd->x() = dr * x;
      xd->y() = dr * y;
      if (m_tangential) {
        const float xy = x * y;
        const float dx = ds[2] * (xy + xy) + ds[3] * (x2 + x2 + x2 + y2);
        const float dy = ds[2] * (x2 + y2 + y2 + y2) + ds[3] * (xy + xy);
        xd->x() = dx + xd->x();
        xd->y() = dy + xd->y();
      }
    }
  }
  inline Point2D GetDistorted(const Point2D &xn) const {
    Point2D xd;
    Distort(xn, &xd);
    return xd;
  }
  bool Undistort(const Point2D &xd, Point2D *xn, LA::AlignedMatrix2x2f *JT = NULL,
                 UndistortionMap *UM = NULL, const bool initialized = false) const;

 protected:

  inline void Set(const float fx, const float fy, const float cx, const float cy) {
    const float fxI = 1.0f / fx, fyI = 1.0f / fy;
    m_f.vset_all_lane(fx, fy, fx, fy);
    m_fI.vset_all_lane(fxI, fyI, fxI, fyI);
    m_c.vset_all_lane(cx, cy, cx, cy);
    m_fx.vdup_all_lane(fx);
    m_fy.vdup_all_lane(fy);
    m_cx.vdup_all_lane(cx);
    m_cy.vdup_all_lane(cy);
    m_fxI.vdup_all_lane(fxI);
    m_fyI.vdup_all_lane(fyI);
    m_fxIcx.vdup_all_lane(fxI * cx);
    m_fyIcy.vdup_all_lane(fyI * cy);
    m_fxx = fx * fx;  m_fxxI = 1.0f / m_fxx;
    m_fxy = fx * fy;  m_fxyI = 1.0f / m_fxy;
    m_fyy = fy * fy;  m_fyyI = 1.0f / m_fyy;
  }

 protected:

  Parameter m_k;
  int m_w, m_h;
  union {
    struct { ubyte m_needRect, m_needUndist, m_fishEye, m_radial6, m_tangential; };
    int m_flag[2];
  };
  xp128f m_f, m_fI, m_c;
  xp128f m_fx, m_fy, m_cx, m_cy;
  xp128f m_fxI, m_fyI, m_fxIcx, m_fyIcy;
  float m_fxx, m_fxxI, m_fxy, m_fxyI, m_fyy, m_fyyI;

};

#ifdef CFG_DEBUG_EIGEN
class EigenIntrinsic : public Eigen::Matrix3f {
 public:
  inline EigenIntrinsic() : Eigen::Matrix3f() {}
  inline EigenIntrinsic(const Eigen::Matrix3f &K) : Eigen::Matrix3f(K) {}
  inline EigenIntrinsic(const Intrinsic &K) : Eigen::Matrix3f() {
    Eigen::Matrix3f &e_K = *this;
    e_K(0, 0) = K.fx(); e_K(0, 1) = 0.0f; e_K(0, 2) = K.cx();
    e_K(1, 0) = 0.0f; e_K(1, 1) = K.fy(); e_K(1, 2) = K.cy();
    e_K(2, 0) = 0.0f; e_K(2, 1) = 0.0f; e_K(2, 2) = 1.0f;
  }
  inline void operator = (const Eigen::Matrix3f &e_K) { *((Eigen::Matrix3f *) this) = e_K; }
  inline EigenPoint2D GetNormaliedToImage(const EigenPoint2D &x) const {
    const Eigen::Matrix3f &e_K = *this;
    const float e_fx = e_K(0, 0), e_fy = e_K(1, 1), e_cx = e_K(0, 2), e_cy = e_K(1, 2);
    return EigenPoint2D(e_fx * x.x() + e_cx, e_fy * x.y() + e_cy);
  }
  template<typename TYPE> inline EigenPoint2D GetImageToNormalized(const TYPE x, const TYPE y) const {
    const Eigen::Matrix3f &e_K = *this;
    const float e_fx = e_K(0, 0), e_fy = e_K(1, 1), e_cx = e_K(0, 2), e_cy = e_K(1, 2);
    return EigenPoint2D((x - e_cx) / e_fx, (y - e_cy) / e_fy);
  }
  inline EigenMatrix2x3f GetNormaliedToImage(const EigenMatrix2x3f &J) const {
    const Eigen::Matrix3f &e_K = *this;
    const float e_fx = e_K(0, 0), e_fy = e_K(1, 1);
    EigenMatrix2x3f e_KJ;
    e_KJ.block<1, 3>(0, 0) = e_fx * J.block<1, 3>(0, 0);
    e_KJ.block<1, 3>(1, 0) = e_fy * J.block<1, 3>(1, 0);
    return e_KJ;
  }
};
#endif

#endif
