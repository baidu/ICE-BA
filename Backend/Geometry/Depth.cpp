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
#include "Depth.h"
#include "Parameter.h"
#include "VectorN.h"

#ifdef CFG_DEBUG
//#define DEPTH_TRI_VERBOSE  1
//#define DEPTH_TRI_VERBOSE  2
#endif
#define DEPTH_TRI_DOG_LEG

namespace Depth {

bool Triangulate(const float w, const LA::AlignedVector3f &t12, const Point2D &x1,
                 const Point2D &x2, InverseGaussian *d, const LA::SymmetricMatrix2x2f *Wx2) {
  float x;
  LA::Vector2f J, e, WJ;
  d->Initialize();
#if 0
  if (Wx2) {
    if (!Triangulate(t12, x1, x2, d)) {
      return false;
    }
  }
#endif
#ifdef DEPTH_TRI_VERBOSE
#if DEPTH_TRI_VERBOSE == 1
  UT::Print("e = %f", sqrtf(fxy * (d->GetProjected(t12, x1) - x2).SquaredLength()));
#endif
#endif
#ifdef DEPTH_TRI_DOG_LEG
  float delta = DEPTH_TRI_DL_RADIUS_INITIAL;
#endif
  const float eps = 0.0f;
  for (int iIter = 0; iIter < DEPTH_TRI_MAX_ITERATIONS; ++iIter) {
    d->Project(t12, x1, e, J);
    e -= x2;
    if (Wx2) {
      LA::SymmetricMatrix2x2f::Ab(*Wx2, J, WJ);
      d->s2() = WJ.Dot(J);
    } else {
      d->s2() = J.Dot(J);
    }
    if (d->s2() <= eps) {
      return false;
    }
    d->s2() = 1.0f / d->s2();
    if (Wx2) {
      x = -d->s2() * WJ.Dot(e);
    } else {
      x = -d->s2() * J.Dot(e);
    }
#ifdef DEPTH_TRI_DOG_LEG
    const float xGN = x;
    const float F = Wx2 ? LA::SymmetricMatrix2x2f::MahalanobisDistance(*Wx2, e) :
                          e.SquaredLength();
    const float dBkp = d->u();
    bool update = true, converge = false;
    for (int iIterDL = 0; iIterDL < DEPTH_TRI_DL_MAX_ITERATIONS; ++iIterDL) {
      if (fabs(xGN) <= delta) {
        x = xGN;
      } else {
        x = x > 0.0f ? delta : -delta;
      }
      d->u() = x + d->u();
      const LA::Vector2f ea = d->GetProjected(t12, x1) - x2;
      const LA::Vector2f ep = e + J * x;
      const float dFa = F - (Wx2 ? LA::SymmetricMatrix2x2f::MahalanobisDistance(*Wx2, ea) :
                             ea.SquaredLength());
      const float dFp = F - (Wx2 ? LA::SymmetricMatrix2x2f::MahalanobisDistance(*Wx2, ep) :
                             ea.SquaredLength());
      const float rho = dFa > 0.0f && dFp > 0.0f ? dFa / dFp : -1.0f;
      if (rho < DEPTH_TRI_DL_GAIN_RATIO_MIN) {
        delta *= DEPTH_TRI_DL_RADIUS_FACTOR_DECREASE;
        if (delta < DEPTH_TRI_DL_RADIUS_MIN) {
          delta = DEPTH_TRI_DL_RADIUS_MIN;
        }
        d->u() = dBkp;
        update = false;
        converge = false;
        continue;
      } else if (rho > DEPTH_TRI_DL_GAIN_RATIO_MAX) {
        delta = std::max(delta, DEPTH_TRI_DL_RADIUS_FACTOR_INCREASE * static_cast<float>(fabs(x)));
        if (delta > DEPTH_TRI_DL_RADIUS_MAX) {
          delta = DEPTH_TRI_DL_RADIUS_MAX;
        }
      }
      update = true;
      converge = fabs(x) < DEPTH_TRI_CONVERGE;
      break;
    }
    if (!update || converge) {
      break;
    }
#else
    d->u() = x + d->u();
    if (fabs(x) < DEPTH_TRI_CONVERGE) {
      break;
    }
#endif
#if defined DEPTH_TRI_VERBOSE && DEPTH_TRI_VERBOSE == 2
    const std::string str = UT::String("%02d  ", iIter);
    if (iIter == 0) {
      UT::PrintSeparator();
      UT::Print("%se = %f\n", std::string(str.size(), ' ').c_str(),
                sqrtf(fxy * (InverseGaussian(0.0f).GetProjected(t12, x1) - x2).SquaredLength()));
    }
    UT::Print("%se = %f  z = %f  x = %f\n", str.c_str(),
              sqrtf(fxy * (d->GetProjected(t12, x1) - x2).SquaredLength()), 1.0f / d->u(), x);
#endif
  }
#if defined DEPTH_TRI_VERBOSE && DEPTH_TRI_VERBOSE == 1
  UT::Print(" --> %f  z = %f  s = %f", sqrtf(fxy * (d->GetProjected(t12, x1) - x2).SquaredLength()),
            1.0f / d->u(), sqrtf(d->s2()));
  if (!d->Valid()) {
    UT::Print("  FAIL");
  }
  UT::Print("\n");
#endif
  d->s2() = DEPTH_VARIANCE_EPSILON + d->s2() * w;
  return d->Valid();
}

bool Triangulate(const float w, const int N, const Measurement *zs, InverseGaussian *d,
                 AlignedVector<float> *work, const bool initialized, const bool robust,
                 float *eAvg) {
  if (!initialized) {
    d->Initialize();
  }
  if (N == 0) {
    return false;
  }
  float a, b, x;
#ifdef DEPTH_TRI_DOG_LEG
  float F, Fa, Fp;
  const int Nx2 = N + N;
  LA::AlignedVectorXf J, e, ep, _w;
  work->Resize((J.BindSize(Nx2) * 3 + (robust ? _w.BindSize(N) : 0)) / sizeof(float));
  J.Bind(work->Data(), Nx2);
  e.Bind(J.BindNext(), Nx2);
  ep.Bind(e.BindNext(), Nx2);
  if (robust) {
    _w.Bind(ep.BindNext(), N);
  }
  AlignedVector<LA::Vector2f> Jis(J.Data(), N, false);
  AlignedVector<LA::Vector2f> eis(e.Data(), N, false);
  AlignedVector<LA::Vector2f> epis(ep.Data(), N, false);
#else
  LA::Vector2f Ji, ei;
#endif
  LA::Vector2f WJi;
#ifdef DEPTH_TRI_DOG_LEG
  float delta = DEPTH_TRI_DL_RADIUS_INITIAL;
#endif
  const float eps = 0.0f;
  for (int iIter = 0; iIter < DEPTH_TRI_MAX_ITERATIONS; ++iIter) {
    a = b = 0.0f;
    for (int i = 0; i < N; ++i) {
      const Measurement &z = zs[i];
#ifdef DEPTH_TRI_DOG_LEG
      LA::Vector2f &Ji = Jis[i], &ei = eis[i];
#endif
      d->Project(*z.m_t, z.m_Rx, ei, Ji);
      ei -= z.m_z;
      LA::SymmetricMatrix2x2f::Ab(z.m_W, Ji, WJi);
      if (robust) {
        const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ei);
        const float wi = ME::Weight<ME::FUNCTION_HUBER>(r2);
        WJi *= wi;
#ifdef DEPTH_TRI_DOG_LEG
        _w[i] = wi;
#endif
      }
      a += WJi.Dot(Ji);
      b += WJi.Dot(ei);
    }
    if (a <= eps) {
      return false;
    }
    d->s2() = 1.0f / a;
    x = -d->s2() * b;
#ifdef DEPTH_TRI_DOG_LEG
    const float xGN = x;
    F = 0.0f;
    for (int i = 0; i < N; ++i) {
      F += LA::SymmetricMatrix2x2f::MahalanobisDistance(zs[i].m_W, eis[i]);
    }
    const float dBkp = d->u();
    bool update = true, converge = false;
    for (int iIterDL = 0; iIterDL < DEPTH_TRI_DL_MAX_ITERATIONS; ++iIterDL) {
      if (fabs(xGN) <= delta) {
        x = xGN;
      } else {
        x = x > 0.0f ? delta : -delta;
      }
      d->u() = x + d->u();

      J.GetScaled(x, ep);
      ep += e;
      Fa = Fp = 0.0f;
      for (int i = 0; i < N; ++i) {
        const Measurement &z = zs[i];
        LA::Vector2f &ei = epis[i];
        const float Fpi = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ei);
        d->Project(*z.m_t, z.m_Rx, ei);
        ei -= z.m_z;
        const float Fai = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ei);
        if (robust) {
          Fp += Fpi * _w[i];
          Fa += Fai * _w[i];
        } else {
          Fp += Fpi;
          Fa += Fai;
        }
      }
      const float dFa = F - Fa, dFp = F - Fp;
      const float rho = dFa > 0.0f && dFp > 0.0f ? dFa / dFp : -1.0f;
      if (rho < DEPTH_TRI_DL_GAIN_RATIO_MIN) {
        delta *= DEPTH_TRI_DL_RADIUS_FACTOR_DECREASE;
        if (delta < DEPTH_TRI_DL_RADIUS_MIN) {
          delta = DEPTH_TRI_DL_RADIUS_MIN;
        }
        d->u() = dBkp;
        update = false;
        converge = false;
        continue;
      } else if (rho > DEPTH_TRI_DL_GAIN_RATIO_MAX) {
        delta = std::max(delta, DEPTH_TRI_DL_RADIUS_FACTOR_INCREASE * static_cast<float>(fabs(x)));
        if (delta > DEPTH_TRI_DL_RADIUS_MAX) {
          delta = DEPTH_TRI_DL_RADIUS_MAX;
        }
      }
      update = true;
      converge = fabs(x) < DEPTH_TRI_CONVERGE;
      break;
    }
    if (!update || converge) {
      break;
    }
#else
    d->u() = x + d->u();
    if (fabs(x) < DEPTH_TRI_CONVERGE) {
      break;
    }
#endif
  }
  d->s2() = DEPTH_VARIANCE_EPSILON + d->s2() * w;
  if (eAvg) {
#ifdef DEPTH_TRI_DOG_LEG
    LA::Vector2f ei;
#endif
    float Se2 = 0.0f;
    for (int i = 0; i < N; ++i) {
      const Measurement &z = zs[i];
      d->Project(*z.m_t, z.m_Rx, ei);
      ei -= z.m_z;
      Se2 += ei.SquaredLength();
    }
    *eAvg = sqrtf(Se2 / N);
  }
  return d->Valid();
}

#ifdef CFG_DEPTH_MAP
extern float DEPTH_MAP_FACTOR_INPUT = DEPTH_MAP_FACTOR;
bool Load(const std::string fileName, CVD::Image<ushort> &Z, CVD::Image<ushort> &ZTmp,
          AlignedVector<float> &work) {
  if (!UT::ImageLoad(fileName, Z, ZTmp)) {
    return false;
  }
  if (DEPTH_MAP_FACTOR == DEPTH_MAP_FACTOR_INPUT) {
    return true;
  }
  const int N = Z.totalsize();
  const float s = DEPTH_MAP_FACTOR / DEPTH_MAP_FACTOR_INPUT;
  const int si = int(s);
  if (s == si) {
    SIMD::Multiply(N, ushort(si), Z.data());
    return true;
  }
  work.Resize(N);
  float *_Z = work.Data();
  SIMD::Multiply(N, s, Z.data(), _Z);
  SIMD::Convert(N, _Z, Z.data());
  return true;
}

bool Save(const std::string fileName, const CVD::Image<float> &D, CVD::Image<float> &Z) {
  if (UT::ImageInvalid(D)) {
    return false;
  }
  Z.resize(D.size());
  SIMD::Convert(D.totalsize(), D.data(), Z.data());
  SIMD::Inverse(Z.totalsize(), Z.data(), DEPTH_MAP_FACTOR / USHRT_MAX);
  UT::ImageSave(fileName, Z);
  return true;
}

void DownSample(const CVD::Image<ushort> &Zs, CVD::Image<ushort> &Zd) {
  if (UT::ImageInvalid(Zs)) {
    UT::ImageInvalidate(Zd);
    return;
  }
  __m128i m1, m2, z1, z2;
  const __m128i m = _mm_setr_epi32(0x0000ffff, 0x0000ffff, 0x0000ffff, 0x0000ffff);
  const __m128i zero = _mm_setzero_si128();
  const int w = Zs.size().x >> 1, h = Zs.size().y >> 1, wF = SIMD_BYTE_FLOOR(w);
  Zd.resize(CVD::ImageRef(wF, h));
  for (int y = 0; y < h; ++y) {
    const int _y1 = y << 1, _y2 = _y1 + 1;
    const __m128i *_z1 = (__m128i *) Zs[_y1], *_z2 = (__m128i *) Zs[_y2];
    __m128i *z = (__m128i *) Zd[y];
    for (int x = 0; x < wF; x += 8, ++z, _z1 += 2, _z2 += 2) {
      m1 = _mm_or_si128(_mm_cmpeq_epi16(_z1[0], zero), _mm_cmpeq_epi16(_z2[0], zero));
      m1 = _mm_or_si128(m1, _mm_srli_si128(m1, 2));
      z1 = _mm_avg_epu16(_z1[0], _z2[0]);
      z1 = _mm_avg_epu16(z1, _mm_srli_si128(z1, 2));
      m2 = _mm_or_si128(_mm_cmpeq_epi16(_z1[1], zero), _mm_cmpeq_epi16(_z2[1], zero));
      m2 = _mm_or_si128(m2, _mm_srli_si128(m2, 2));
      z2 = _mm_avg_epu16(_z1[1], _z2[1]);
      z2 = _mm_avg_epu16(z2, _mm_srli_si128(z2, 2));
      *z = _mm_packus_epi32(_mm_and_si128(_mm_andnot_si128(m1, z1), m),
                            _mm_and_si128(_mm_andnot_si128(m2, z2), m));
    }
#if 0
//#ifdef 1
    for (int x = 0; x < wF; ++x) {
      ushort _z1;
      const int _x1 = x << 1, _x2 = _x1 + 1;
      const ushort z11 = Zs[_y1][_x1], z12 = Zs[_y2][_x1], z21 = Zs[_y1][_x2], z22 = Zs[_y2][_x2];
      if (z11 == 0 || z12 == 0 || z21 == 0 || z22 == 0) {
        _z1 = 0;
      } else {
        _z1 = ushort((int(z11) + int(z12) + int(z21) + int(z22)) >> 2);
      }
      const ushort _z2 = Zd[y][x];
      UT_ASSERT(_z1 == 0 && _z2 == 0 || _z1 == _z2 || _z2 == _z1 + 1);
    }
#endif
  }
}

void LoadDepths(const CVD::Image<ushort> &Z, const Point2D &x, ushort *zs) {
#ifdef CFG_DEBUG
  UT_ASSERT(UT::ImageValid(Z));
#endif
  const int ix = int(x.x()), x1 = ix - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF;
  const int iy = int(x.y()), y1 = iy - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF,
            y2 = y1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
#ifdef CFG_DEBUG
  UT_ASSERT(x1 >= 0 && y1 >= 0 && x1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE <= Z.size().x &&
            y2 <= Z.size().y);
#endif
  const int size = sizeof(ushort) * DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
  for (int y = y1; y < y2; ++y, zs += DEPTH_MAP_SMOOTHNESS_PATCH_SIZE) {
    memcpy(zs, Z[y] + x1, size);
  }
}

void LoadDepths(const CVD::Image<ushort> &Z, const std::vector<LA::Vector2us> &xs,
                AlignedVector<float> &ds, AlignedVector<float> &work) {
#ifdef CFG_DEBUG
  UT_ASSERT(UT::ImageValid(Z));
#endif
  const int Nx = int(xs.size()), NxC = SIMD_SHORT_CEIL(Nx);
  work.Resize(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS + NxC * sizeof(ushort) / sizeof(float) + Nx);
  AlignedVector<ushort> zis((ushort *) work.Data(), DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, false);
  AlignedVector<ushort> zs((ushort *) (work.Data() + DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS), Nx, false);
  zs.Resize(Nx);
  const int size = sizeof(ushort) * DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
  for (int i = 0; i < Nx; ++i) {
    const LA::Vector2us &x = xs[i];
    const int x1 = x.x() - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF;
    const int y1 = x.y() - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF,
              y2 = y1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
#ifdef CFG_DEBUG
    UT_ASSERT(x1 >= 0 && y1 >= 0 && x1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE <= Z.size().x &&
              y2 <= Z.size().y);
#endif
    ushort *z = zis.Data();
    for (int y = y1; y < y2; ++y, z += DEPTH_MAP_SMOOTHNESS_PATCH_SIZE)
      memcpy(z, Z[y] + x1, size);
    const ushort _z = zis[DEPTH_MAP_SMOOTHNESS_PATCH_CENTER];
    if (SIMD::ExistEqual<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zis.Data(), 0)
        || SIMD::ExistGreater<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zis.Data(),
                                     _z + DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION)
        || _z > DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION &&
        SIMD::ExistLess<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zis.Data(),
                               _z - DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION)) {
      zs[i] = 0;
    } else {
      zs[i] = _z;
    }
  }
  ds.Resize(Nx);
  SIMD::Convert(Nx, zs.Data(), ds.Data());
  SIMD::Inverse(Nx, ds.Data(), DEPTH_MAP_FACTOR, true);
}

void LoadDepths(const CVD::Image<ushort> &Z, const int Nx, const LA::Vector2us *xs,
                std::vector<Point> &xds, AlignedVector<float> &work) {
#ifdef CFG_DEBUG
  UT_ASSERT(UT::ImageValid(Z));
#endif
  const int NxC = SIMD_SHORT_CEIL(Nx);
  work.Resize(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS + NxC * sizeof(ushort) / sizeof(float) + Nx);
  AlignedVector<ushort> zis((ushort *) work.Data(), DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, false);
  AlignedVector<ushort> zs((ushort *) (work.Data() + DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS), Nx, false);
  LA::AlignedVectorXf ds((float *) (zs.Data() + NxC), Nx, false);
  zs.Resize(Nx);
  xds.resize(Nx);

  int i, j;
  const int size = sizeof(ushort) * DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
  for (i = j = 0; i < Nx; ++i) {
    const LA::Vector2us &x = xs[i];
    const int x1 = x.x() - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF;
    const int y1 = x.y() - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF,
              y2 = y1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
#ifdef CFG_DEBUG
    UT_ASSERT(x1 >= 0 && y1 >= 0 && x1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE <= Z.size().x &&
              y2 <= Z.size().y);
#endif
    ushort *z = zis.Data();
    for (int y = y1; y < y2; ++y, z += DEPTH_MAP_SMOOTHNESS_PATCH_SIZE)
      memcpy(z, Z[y] + x1, size);
    const ushort _z = zis[DEPTH_MAP_SMOOTHNESS_PATCH_CENTER];
    if (SIMD::ExistEqual<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zis.Data(), 0)
        || SIMD::ExistGreater<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zis.Data(),
                                     _z + DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION)
        || _z > DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION &&
        SIMD::ExistLess<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zis.Data(),
                               _z - DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION)) {
      continue;
    }
    zs[j] = _z;
    xds[j].m_x = x;
    ++j;
  }
  const int Nxd = j;
  zs.Resize(Nxd);
  ds.Resize(Nxd);
  SIMD::Convert(Nxd, zs.Data(), ds.Data());
  ds.MakeInverse(DEPTH_MAP_FACTOR);
  xds.resize(Nxd);
  for (int i = 0; i < Nxd; ++i) {
    xds[i].m_d = ds[i];
  }
}

float InterpolateDepth(const CVD::Image<ushort> &Z, const Point2D &x, AlignedVector<float> &work) {
#ifdef CFG_DEBUG
  UT_ASSERT(UT::ImageValid(Z));
#endif
  const int ix = int(x.x()), x1 = ix - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF;
  const int iy = int(x.y()), y1 = iy - DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF,
            y2 = y1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
#ifdef CFG_DEBUG
  UT_ASSERT(x1 >= 0 && y1 >= 0 && x1 + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE <= Z.size().x &&
            y2 <= Z.size().y);
#endif
  work.Resize(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS);
  const int size = sizeof(ushort) * DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
  ushort *zs = (ushort *) work.Data(), *z = zs;
  for (int y = y1; y < y2; ++y, z += DEPTH_MAP_SMOOTHNESS_PATCH_SIZE) {
    memcpy(z, Z[y] + x1, size);
  }
  const ushort _z = zs[DEPTH_MAP_SMOOTHNESS_PATCH_CENTER];
  if (SIMD::ExistEqual<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zs, 0)
      || SIMD::ExistGreater<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zs,
                                   _z + DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION)
      || _z > DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION &&
      SIMD::ExistLess<ushort>(DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS, zs,
                             _z - DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION)) {
    return 0.0f;
  }
  __m128 w;
  UT::ImageInterpolateWeight(x.x() - ix, x.y() - iy, w);
  const __m128 ds = _mm_div_ps(_mm_set1_ps(DEPTH_MAP_FACTOR), _mm_setr_ps(float(_z),
                                                                          float(zs[DEPTH_MAP_SMOOTHNESS_PATCH_CENTER + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE]),
                                                                          float(zs[DEPTH_MAP_SMOOTHNESS_PATCH_CENTER + 1]),
                                                                          float(zs[DEPTH_MAP_SMOOTHNESS_PATCH_CENTER + DEPTH_MAP_SMOOTHNESS_PATCH_SIZE + 1])));
  return SIMD::Sum(_mm_mul_ps(w, ds));
}
#endif
}
