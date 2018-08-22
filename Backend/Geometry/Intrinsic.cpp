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
#include "stdafx.h"
#include "Intrinsic.h"
#include "Parameter.h"

#ifdef CFG_DEBUG
//#define FTR_UNDIST_VERBOSE  1
//#define FTR_UNDIST_VERBOSE  2
#endif
#define FTR_UNDIST_DOG_LEG

void Intrinsic::UndistortionMap::Set(const Intrinsic &K) {
  const float sx = float(FTR_UNDIST_LUT_SIZE - 1) / K.w();
  const float sy = float(FTR_UNDIST_LUT_SIZE - 1) / K.h();
  m_fx = sx * K.k().m_fx; m_fy = sy * K.k().m_fy;
  m_cx = sx * K.k().m_cx; m_cy = sy * K.k().m_cy;
  const float fxI = 1.0f / m_fx, fyI = 1.0f / m_fy;
  m_xns.Resize(FTR_UNDIST_LUT_SIZE, FTR_UNDIST_LUT_SIZE);

#ifdef FTR_UNDIST_VERBOSE
  UT::PrintSeparator();
#endif
  Point2D xd, xn;
  float v;
  int x[2] = {FTR_UNDIST_LUT_SIZE / 2, FTR_UNDIST_LUT_SIZE / 2};
  int b[2][2] = {{x[0] - 1, x[0] + 1}, {x[1] - 1, x[1] + 1}};
  const int d[2] = {-1, 1};
  const int N = m_xns.Size();
  for(int i = 0, ix = 0, id = 0; i < N; ++i) {
    xd.x() = fxI * (x[0] - m_cx);
    xd.y() = fyI * (x[1] - m_cy);
    if (i == 0) {
      xn = xd;
    } else {
      xn = xd * v;
    }
    if (!K.Undistort(xd, &xn, NULL, NULL, true)) {
      exit(0);
    }
    v = sqrtf(xn.SquaredLength() / xd.SquaredLength());
//#ifdef CFG_DEBUG
#if 0
    xn.x() = v;
#endif
    m_xns[x[1]][x[0]] = xn;
//#ifdef CFG_DEBUG
#if 0
    //UT::Print("%d %d\n", x[0], x[1]);
    const float s = 0.5f;
    const int _w = K.w(), _h = K.h();
    const float _fx = K.fx() * s, _fy = K.fy() * s;
    const float _cx = (_w - 1) * 0.5f, _cy = (_h - 1) * 0.5f;
    CVD::Image<CVD::Rgb<ubyte> > I;
    I.resize(CVD::ImageRef(_w, _h));
    I.zero();
    UT::ImageDrawCross(I, int(_fx * xd.x() + _cx), int(_fy * xd.y() + _cy), 5, CVD::Rgb<ubyte>(255, 0, 0));
    UT::ImageDrawCross(I, int(_fx * xn.x() + _cx), int(_fy * xn.y() + _cy), 5, CVD::Rgb<ubyte>(0, 255, 0));
    UT::ImageSave(UT::String("D:/tmp/test%04d.jpg", i), I);
#endif
    if (x[ix] == b[ix][id]) {
      b[ix][id] += d[id];
      ix = 1 - ix;
      if (ix == 0) {
        id = 1 - id;
      }
    }
    x[ix] += d[id];
  }
#ifdef FTR_UNDIST_VERBOSE
  UT::PrintSeparator();
#endif
}

bool Intrinsic::Undistort(const Point2D &xd, Point2D *xn, LA::AlignedMatrix2x2f *JT,
                          UndistortionMap *UM, const bool initialized) const {
#ifdef CFG_DEBUG
  if (FishEye()) {
    UT::Error("TODO (haomin)\n");
  }
#endif
  if (!NeedUndistortion()) {
    *xn = xd;
    JT->MakeIdentity();
    return true;
  }
  float dr, j, dx2;
  LA::AlignedMatrix2x2f J;
  LA::SymmetricMatrix2x2f A;
  LA::AlignedMatrix2x2f AI;
  LA::Vector2f e, dx;
#ifdef FTR_UNDIST_DOG_LEG
  float dx2GN, dx2GD, delta2, beta;
  LA::Vector2f dxGN, dxGD;
  bool update, converge;
#endif
  if (UM) {
    if (UM->Empty()) {
      UM->Set(*this);
    }
    *xn = UM->Get(xd);
//#ifdef CFG_DEBUG
#if 0
    *xn = xd * xn->x();
#endif
  } else if (!initialized) {
    *xn = xd;
  }
#if defined FTR_UNDIST_VERBOSE && FTR_UNDIST_VERBOSE == 1
  const Point2D _xd = m_k.GetNormalizedToImage(xd);
  UT::Print("x = %03d %03d", int(_xd.x() + 0.5f), int(_xd.y() + 0.5f));
  UT::Print("  e = %f", sqrtf((GetDistorted(*xn) - xd).SquaredLength() * m_k.m_fx * m_k.m_fy));
#endif
  const float *ds = m_k.m_ds, *jds = m_k.m_jds;
  const float dx2Conv = FTR_UNDIST_CONVERGE * fxyI();
  //const float dx2Conv = FTR_UNDIST_CONVERGE * m_k.m_fxyI;
#ifdef FTR_UNDIST_DOG_LEG
  delta2 = FTR_UNDIST_DL_RADIUS_INITIAL;
#endif
  for (int iIter = 0; iIter < FTR_UNDIST_MAX_ITERATIONS; ++iIter) {
#if 0
//#if 1
    if (UT::Debugging()) {
      UT::Print("%d %e %e\n", iIter, xn->x(), xn->y());
    }
#endif
    const float x = xn->x(), x2 = x * x, y = xn->y(), y2 = y * y, xy = x * y;
    const float r2 = x2 + y2, r4 = r2 * r2, r6 = r2 * r4;
    dr = ds[4] * r6 + ds[1] * r4 + ds[0] * r2 + 1.0f;
    j = jds[4] * r4 + jds[1] * r2 + ds[0];
    if (m_radial6) {
      const float dr2I = 1.0f / (ds[7] * r6 + ds[6] * r4 + ds[5] * r2 + 1.0f);
      dr *= dr2I;
      j = (j - (jds[7] * r4 + jds[6] * r2 + ds[5]) * dr) * dr2I;
    }
    e.x() = dr * x;
    e.y() = dr * y;
    J.m00() = j * (x2 + x2) + dr;
    J.m01() = j * (xy + xy);
    J.m11() = j * (y2 + y2) + dr;
//#ifdef CFG_DEBUG
#if 0
    J.m00() = 1 - 3 * x2 - y2;
    J.m01() = -2 * xy;
    J.m11() = 1 - x2 - 3 * y2;
#endif
    if (m_tangential) {
      const float dx = jds[2] * xy + ds[3] * (r2 + x2 + x2);
      const float dy = jds[3] * xy + ds[2] * (r2 + y2 + y2);
      e.x() = dx + e.x();
      e.y() = dy + e.y();
      const float d2x = jds[2] * x, d2y = jds[2] * y;
      const float d3x = jds[3] * x, d3y = jds[3] * y;
      J.m00() = d3x + d3x + d3x + d2y + J.m00();
      J.m01() = d2x + d3y + J.m01();
      J.m11() = d3x + d2y + d2y + d2y + J.m11();
    }
    J.m10() = J.m01();
    e -= xd;
    LA::SymmetricMatrix2x2f::AAT(J, A);
//#ifdef CFG_DEBUG
#if 0
    A.Set(J.m00(), J.m01(), J.m11());
#endif
    const LA::Vector2f b = J * e;
    if (!A.GetInverse(AI)) {
      //return false;
      break;
    }
    LA::AlignedMatrix2x2f::Ab(AI, b, dx);
    dx.MakeMinus();
    dx2 = dx.SquaredLength();
#ifdef FTR_UNDIST_DOG_LEG
    if (!UM) {
      dxGN = dx;
      dx2GN = dx2;
      dx2GD = 0.0f;
      const float F = e.SquaredLength();
      const Point2D xnBkp = *xn;
      update = true;
      converge = false;
      for (int iIterDL = 0; iIterDL < FTR_UNDIST_DL_MAX_ITERATIONS; ++iIterDL) {
        if (dx2GN > delta2 && dx2GD == 0.0f) {
          const float bl = sqrtf(b.SquaredLength());
          const LA::Vector2f g = b * (1.0f / bl);
          const LA::Vector2f Ag = A * g;
          const float xl = bl / g.Dot(Ag);
          g.GetScaled(-xl, dxGD);
          dx2GD = xl * xl;
#ifdef CFG_DEBUG
         UT::AssertEqual(dxGD.SquaredLength(), dx2GD);
#endif
        }
        if (dx2GN <= delta2) {
          dx = dxGN;
          dx2 = dx2GN;
          beta = 1.0f;
        } else if (dx2GD >= delta2) {
          if (delta2 == 0.0f) {
            dx = dxGD;
            dx2 = dx2GD;
          } else {
            dxGD.GetScaled(sqrtf(delta2 / dx2GD), dx);
            dx2 = delta2;
          }
          beta = 0.0f;
        } else {
          const LA::Vector2f v = dxGN - dxGD;
          const float d = dxGD.Dot(v), v2 = v.SquaredLength();
          //beta = float((-d + sqrt(double(d) * d + (delta2 - dx2GD) * double(v2))) / v2);
          beta = (-d + sqrtf(d * d + (delta2 - dx2GD) * v2)) / v2;
          dx = dxGD;
          dx += v * beta;
          dx2 = delta2;
        }
        *xn += dx;
        const float dFa = F - (GetDistorted(*xn) - xd).SquaredLength();
        const float dFp = F - (e + J * dx).SquaredLength();
        const float rho = dFa > 0.0f && dFp > 0.0f ? dFa / dFp : -1.0f;
        if (rho < FTR_UNDIST_DL_GAIN_RATIO_MIN) {
          delta2 *= FTR_UNDIST_DL_RADIUS_FACTOR_DECREASE;
          if (delta2 < FTR_UNDIST_DL_RADIUS_MIN) {
            delta2 = FTR_UNDIST_DL_RADIUS_MIN;
          }
          *xn = xnBkp;
          update = false;
          converge = false;
          continue;
        } else if (rho > FTR_UNDIST_DL_GAIN_RATIO_MAX) {
          delta2 = std::max(delta2, FTR_UNDIST_DL_RADIUS_FACTOR_INCREASE * dx2);
          if (delta2 > FTR_UNDIST_DL_RADIUS_MAX) {
            delta2 = FTR_UNDIST_DL_RADIUS_MAX;
          }
        }
        update = true;
        converge = dx2 < dx2Conv;
        break;
      }
      if (!update || converge) {
        break;
      }
    } else
#endif
    {
      *xn += dx;
      if (dx2 < dx2Conv) {
        break;
      }
    }
#if defined FTR_UNDIST_VERBOSE && FTR_UNDIST_VERBOSE == 2
    const std::string str = UT::String("%02d  ", iIter);
    if (iIter == 0) {
      UT::PrintSeparator();
      m_k.GetNormalizedToImage(xd).Print(std::string(str.size(), ' ') + "x = ", false, true);
    }
    GetNormalizedToImage(xnBkp).Print(str + "x = ", false, false);
    UT::Print("  e = %f  dx = %f  beta = %f\n", sqrtf(F * fxy()), sqrtf(dx2 * fxy()), beta);
#endif
  }
  if (JT) {
    J.GetTranspose(*JT);
  }
#if defined FTR_UNDIST_VERBOSE && FTR_UNDIST_VERBOSE == 1
  UT::Print(" --> %f\n", sqrtf((GetDistorted(*xn) - xd).SquaredLength() * m_k.m_fx * m_k.m_fy));
#endif
  return true;
}
