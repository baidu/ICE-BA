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

//#ifdef CFG_DEBUG
#if 1
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
  const float f = 500.0f;
#if DEPTH_TRI_VERBOSE == 1
  UT::Print("e = %f", f * sqrtf((d->GetProjected(t12, x1) - x2).SquaredLength()));
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
                f * sqrtf((InverseGaussian(0.0f).GetProjected(t12, x1) - x2).SquaredLength()));
    }
    UT::Print("%se = %f  z = %f  x = %f\n", str.c_str(),
              f * sqrtf((d->GetProjected(t12, x1) - x2).SquaredLength()), 1.0f / d->u(), x);
#endif
  }
#if defined DEPTH_TRI_VERBOSE && DEPTH_TRI_VERBOSE == 1
  UT::Print(" --> %f  z = %f  s = %f", f * sqrtf((d->GetProjected(t12, x1) - x2).SquaredLength()),
            1.0f / d->u(), sqrtf(d->s2()));
  if (!d->Valid()) {
    UT::Print("  FAIL");
  }
  UT::Print("\n");
#endif
  d->s2() = DEPTH_VARIANCE_EPSILON + d->s2() * w;
  return d->Valid();
}

float ComputeError(const int N, const Measurement *zs, const InverseGaussian &d) {
  LA::Vector2f ei;
  float Se2 = 0.0f;
  for (int i = 0; i < N; ++i) {
    const Measurement &z = zs[i];
    d.Project(*z.m_t, z.m_Rx, ei);
    ei -= z.m_z;
    Se2 += ei.SquaredLength();
  }
  return sqrtf(Se2 / N);
}

bool Triangulate(const float w, const int N, const Measurement *zs, InverseGaussian *d,
                 AlignedVector<float> *work, const bool initialized,
                 float *eAvg) {
  if (N == 0) {
    return false;
  }
  if (!initialized) {
    d->Initialize();
  }
  float a, b, x;
#ifdef DEPTH_TRI_DOG_LEG
  float F, Fa, Fp;
  const int Nx2 = N + N;
  LA::AlignedVectorXf J, e, ep, _w;
  work->Resize((J.BindSize(Nx2) * 3 + (DEPTH_TRI_ROBUST ? _w.BindSize(N) : 0)) / sizeof(float));
  J.Bind(work->Data(), Nx2);
  e.Bind(J.BindNext(), Nx2);
  ep.Bind(e.BindNext(), Nx2);
  if (DEPTH_TRI_ROBUST) {
    _w.Bind(ep.BindNext(), N);
  }
  AlignedVector<LA::Vector2f> Jis(J.Data(), N, false);
  AlignedVector<LA::Vector2f> eis(e.Data(), N, false);
  AlignedVector<LA::Vector2f> epis(ep.Data(), N, false);
#else
  LA::Vector2f Ji, ei;
#endif
  LA::Vector2f WJi;
#ifdef DEPTH_TRI_VERBOSE
  const float f = 500.0f;
#if DEPTH_TRI_VERBOSE == 1
  UT::Print("e = %f", f * ComputeError(N, zs, *d));
#else if DEPTH_TRI_VERBOSE == 2
  const InverseGaussian d0 = *d;
#endif
#endif
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
      if (DEPTH_TRI_ROBUST) {
        const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ei);
        const float wi = ME::Weight<ME::FUNCTION_HUBER>(r2);
        WJi *= wi;
#ifdef DEPTH_TRI_DOG_LEG
        _w[i] = wi;
#endif
#if 0
//#if 1
        UT::Print("%d %f\n", i, wi);
#endif
      }
      a += WJi.Dot(Ji);
      b += WJi.Dot(ei);
#if 0
//#if 1
      UT::Print("%d %f %f\n", i, a, b);
#endif
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
        if (DEPTH_TRI_ROBUST) {
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
#if defined DEPTH_TRI_VERBOSE && DEPTH_TRI_VERBOSE == 2
    const std::string str = UT::String("%02d  ", iIter);
    if (iIter == 0) {
      UT::PrintSeparator();
      UT::Print("%se = %f  z = %f\n", std::string(str.size(), ' ').c_str(),
                f * ComputeError(N, zs, d0), 1.0f / d0.u());
    }
    UT::Print("%se = %f  z = %f  x = %f\n", str.c_str(), f * ComputeError(N, zs, *d),
              1.0f / d->u(), x);
#endif
  }
#if defined DEPTH_TRI_VERBOSE && DEPTH_TRI_VERBOSE == 1
  UT::Print(" --> %f  z = %f  s = %f", f * ComputeError(N, zs, *d), 1.0f / d->u(), sqrtf(d->s2()));
  if (!d->Valid()) {
    UT::Print("  FAIL");
  }
  UT::Print("\n");
#endif
  d->s2() = DEPTH_VARIANCE_EPSILON + d->s2() * w;
  if (eAvg) {
    *eAvg = ComputeError(N, zs, *d);
  }
  return d->Valid();
}
}
