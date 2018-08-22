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
#ifndef _VECTOR_N_H_
#define _VECTOR_N_H_

#include "AlignedVector.h"
#include "Vector4.h"
#include "Vector6.h"
#include "Vector8.h"
#include "Vector9.h"
#include "Vector12.h"

namespace LA {

template<int N>
class SIMD_ALIGN_DECLSPEC AlignedVectorNf {
 public:
  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline const float& operator[] (const int i) const { return m_data[i]; }
  inline       float& operator[] (const int i)       { return m_data[i]; }
  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline void operator += (const AlignedVectorNf<N> &other) {
    // TODO (yanghongtian) : rooms for optimizations
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      m_data4[i] += other.m_data4[i];
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      m_data4[NF4] += other.m_data4[NF4];
    } else {
      for (int i = NF; i < N; ++i) {
        m_data[i] += other.m_data[i];
      }
    }
  }
  inline void operator -= (const AlignedVectorNf<N> &other) {
    // TODO (yanghongtian) : rooms for optimizations
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      m_data4[i] -= other.m_data4[i];
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      m_data4[NF4] -= other.m_data4[NF4];
    } else {
      for (int i = NF; i < N; ++i) {
        m_data[i] -= other.m_data[i];
      }
    }
  }
  inline AlignedVectorNf<N> operator - (const AlignedVectorNf<N> &b) const {
    AlignedVectorNf<N> amb;
    LA::AlignedVectorNf<N>::amb(*this, b, amb);
    return amb;
  }
  inline void operator *= (const xp128f &s) {
    // TODO (yanghongtian) : rooms for optimizations
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      m_data4[i] *= s;
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      m_data4[NF4] *= s;
    } else {
      for (int i = NF; i < N; ++i) {
        m_data[i] *= s[0];
      }
    }
  }

  inline void Set(const float *v) {
    memcpy(this, v, sizeof(float) * N);
  }

  inline void GetBlock(const int i, AlignedVector4f &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i + 4 <= N);
#endif
    memcpy(&v, &m_data[i], 16);
  }

  inline void SetBlock(const int i, const Vector3f &v) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i + 3 <= N);
#endif
    memcpy(&m_data[i], &v, 12);
  }
  inline void SetBlock(const int i, const AlignedVector4f &v) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i + 4 <= N);
#endif
    memcpy(&m_data[i], &v, 16);
  }
  inline void SetBlock(const int i, const Vector6f &v) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i + 6 <= N);
#endif
    memcpy(&m_data[i], &v, 24);
  }
  inline void SetBlock(const int i, const AlignedVector8f &v) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i + 8 <= N);
#endif
    memcpy(&m_data[i], &v, 32);
  }

  inline void MakeZero() {
    memset(this, 0, sizeof(AlignedVectorNf<N>));
  }
  inline void MakeMinus() {
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      m_data4[i].vmake_minus();
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      m_data4[NF4].vmake_minus();
    } else {
      for (int i = NF; i < N; ++i) {
        m_data[i] = -m_data[i];
      }
    }
  }

  inline float Dot(const AlignedVector3f &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 3);
#endif
    return (m_data4[0] * v.v012r()).vsum_012();
  }
  inline float Dot(const xp128f &v0123, const float v4, const float v5) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 6);
#endif
    return (m_data4[0] * v0123).vsum_all() + m_data[4] * v4 + m_data[5] * v5;
  }
  inline float Dot(const AlignedVector6f &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 6);
#endif
    return (m_data4[0] * v.v0123()).vsum_all() + m_data[4] * v.v4() + m_data[5] * v.v5();
  }
  inline float Dot(const AlignedVector9f &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 9);
#endif
    return (m_data4[0] * v.v0123() + m_data4[1] * v.v4567()).vsum_all() + m_data[8] * v.v8();
  }
  inline float Dot(const AlignedVector12f &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 12);
#endif
    return (m_data4[0] * v.v0123() + m_data4[1] * v.v4567() +
            m_data4[2] * v.v_8_9_10_11()).vsum_all();
  }
  inline float Dot(const AlignedVectorNf<N> &v) const {
    // TODO (yanghongtian) : rooms for optimizations
    xp128f s = xp128f::get(0.0f);
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      s += m_data4[i] * v.m_data4[i];
    }
    float _s = s.vsum_all();
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      _s += (m_data4[NF4] * v.m_data4[NF4]).vsum_012();
    } else {
      for (int i = NF; i < N; ++i) {
        _s += m_data[i] * v.m_data[i];
      }
    }
    return _s;
  }

  inline void GetScaled(const AlignedVector6f &s, AlignedVectorNf<N> &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 6);
#endif
    v.m_data4[0] = m_data4[0] * s.v0123();
    v.m_data[4] = m_data[4] * s.v4();
    v.m_data[5] = m_data[5] * s.v5();
  }
  inline void GetScaled(const AlignedVector9f &s, AlignedVectorNf<N> &v) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 9);
#endif
    v.m_data4[0] = m_data4[0] * s.v0123();
    v.m_data4[1] = m_data4[1] * s.v4567();
    v.m_data[8] = m_data[8] * s.v8();
  }
  inline void Scale(const float s) { const xp128f _s = xp128f::get(s); Scale(_s); }
  inline void Scale(const xp128f &s) {
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      m_data4[i] = s * m_data4[i];
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      m_data4[NF4] = s * m_data4[NF4];
    } else {
      for (int i = NF; i < N; ++i) {
        m_data[i] = s[0] * m_data[i];
      }
    }
  }
  inline void GetScaled(const xp128f &s, AlignedVectorNf<N> &v) const {
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      v.m_data4[i] = s * m_data4[i];
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      v.m_data4[NF4] = s * m_data4[NF4];
    } else {
      for (int i = NF; i < N; ++i) {
        v.m_data[i] = s[0] * m_data[i];
      }
    }
  }

  inline void Print(const bool e = false) const {
    for (int i = 0; i < N; ++i) {
      if (e) {
        UT::Print("%e ", m_data[i]);
      } else {
        UT::Print("%f ", m_data[i]);
      }
    }
    UT::Print("\n");
  }

  inline bool AssertEqual(const AlignedVectorNf<N> &v,
                          const int verbose = 1,
                          const std::string str = "",
                          const float epsAbs = 0.0f,
                          const float epsRel = 0.0f) const {
    bool equal = true;
    for (int i = 0; i < N && equal; ++i) {
      equal = UT::AssertEqual(m_data[i], v[i], verbose,
                              UT::String("%s[%d]", str.c_str(), i),
                              epsAbs, epsRel);
    }
    if (equal) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      v.Print(verbose > 1);
      const AlignedVectorNf<N> e = *this - v;
      UT::PrintSeparator();
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    bool zero = true;
    //const int verbose1 = verbose & 3, verbose2 = verbose >> 2;
    const int verbose1 = verbose, verbose2 = verbose;
    for (int i = 0; i < N && zero; ++i) {
      zero = UT::AssertZero(m_data[i], verbose1, UT::String("%s[%d]", str.c_str(), i),
                            epsAbs, epsRel);
    }
    if (zero) {
      return true;
    } else if (verbose2) {
      UT::PrintSeparator();
      Print(verbose2 > 1);
    }
    return false;
  }

  inline void Random(const float vMax) {
    UT::Random(m_data, N, -vMax, vMax);
  }

  static inline void apb(const AlignedVectorNf<N> &a, const AlignedVectorNf<N> &b,
                         AlignedVectorNf<N> &apb) {
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      apb.m_data4[i] = a.m_data4[i] + b.m_data4[i];
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      apb.m_data4[NF4] = a.m_data4[NF4] + b.m_data4[NF4];
    } else {
      for (int i = NF; i < N; ++i) {
        apb.m_data[i] = a.m_data[i] + b.m_data[i];
      }
    }
  }
  static inline void amb(const AlignedVectorNf<N> &a, const AlignedVectorNf<N> &b,
                         AlignedVectorNf<N> &amb) {
    const int NF4 = N >> 2;
    for (int i = 0; i < NF4; ++i) {
      amb.m_data4[i] = a.m_data4[i] - b.m_data4[i];
    }
    const int NF = NF4 << 2;
    if (N - NF == 3) {
      amb.m_data4[NF4] = a.m_data4[NF4] - b.m_data4[NF4];
    } else {
      for (int i = NF; i < N; ++i) {
        amb.m_data[i] = a.m_data[i] - b.m_data[i];
      }
    }
  }

 public:
  union {
    float m_data[N];
    xp128f m_data4[(N + 3) >> 2];
  };
};

class SIMD_ALIGN_DECLSPEC AlignedVector13f : public AlignedVectorNf<13> {
 public:
  inline void Set(const float v0, const Vector6f &v1, const Vector6f &v2) {
    m_data[0] = v0;
    SetBlock(1, v1);
    SetBlock(7, v2);
  }
  inline void GetScaled(const float s, AlignedVectorNf<13> &v) const {
    const xp128f _s = xp128f::get(s);
    GetScaled(_s, v);
  }
  inline void GetScaled(const xp128f &s, AlignedVectorNf<13> &other) const {
    other.m_data4[0] = m_data4[0] * s;
    other.m_data4[1] = m_data4[1] * s;
    other.m_data4[2] = m_data4[2] * s;
    other.m_data[12] = m_data[12] * s[0];
  }
};

class SIMD_ALIGN_DECLSPEC AlignedVector14f : public AlignedVectorNf<14> {
 public:
  inline void Set(const AlignedVector13f &v0, const float v1) {
    memcpy(m_data, v0.m_data, sizeof(float) * 13);
    m_data[13] = v1;
  }
  inline void GetScaled(const float s, AlignedVectorNf<14> &v) const {
    const xp128f _s = xp128f::get(s);
    GetScaled(_s, v);
  }
  inline void GetScaled(const xp128f &s, AlignedVectorNf<14> &other) const {
    other.m_data4[0] = m_data4[0] * s;
    other.m_data4[1] = m_data4[1] * s;
    other.m_data4[2] = m_data4[2] * s;
    other.m_data[12] = m_data[12] * s[0];
    other.m_data[13] = m_data[13] * s[0];
  }
 public:
  static inline void AddsATo(const float s, const AlignedVectorNf<14> &a, AlignedVectorNf<14> &sa) {
    xp128f _s; _s.vdup_all_lane(s);
    AddsATo(_s, a, sa);
  }
  static inline void AddsATo(const xp128f &s, const AlignedVectorNf<14> &a, AlignedVectorNf<14> &sa) {
    sa.m_data4[0] += a.m_data4[0] * s;
    sa.m_data4[1] += a.m_data4[1] * s;
    sa.m_data4[2] += a.m_data4[2] * s;
    sa.m_data[12] += a.m_data[12] * s[0];
    sa.m_data[13] += a.m_data[13] * s[0];
  }
};

class SIMD_ALIGN_DECLSPEC AlignedVector18f : public AlignedVectorNf<18> {
};

template<typename TYPE>
class AlignedVectorX : public AlignedVector<TYPE> {

 public:

  inline AlignedVectorX() : AlignedVector<TYPE>() {}
  inline AlignedVectorX(void *V, const int N, const bool own = true) :
                        AlignedVector<TYPE>(V, N, own) {}
  inline AlignedVectorX(const AlignedVectorX<TYPE> &V) { *this = V; }

  inline void operator = (const AlignedVectorX<TYPE> &V) { Set(V); }
  //inline void operator = (AlignedVectorX<TYPE> &V) { Bind(V.Data(), V.Size()); }

  inline void operator += (const TYPE v) { SIMD::Add(this->Size(), v, this->Data()); }
  inline void operator += (const AlignedVectorX<TYPE> &V) {
    const int N = this->Size();
#ifdef CFG_DEBUG
    UT_ASSERT(V.Size() == N);
#endif
    SIMD::Add(N, V.Data(), this->Data());
  }
  inline void operator -= (const TYPE v) { SIMD::Add(this->Size(), -v, this->Data()); }
  inline void operator -= (const TYPE *V) { SIMD::Subtract(this->Size(), this->Data(), V); }
  inline void operator -= (const AlignedVectorX<TYPE> &V) {
    const int N = this->Size();
#ifdef CFG_DEBUG
    UT_ASSERT(V.Size() == N);
#endif
    SIMD::Subtract(N, this->Data(), V.Data());
  }
  inline AlignedVectorX<TYPE> operator - (const AlignedVectorX<TYPE> &V) const {
    AlignedVectorX<TYPE> Vamb;
    AmB(*this, V, Vamb);
    return Vamb;
  }
  inline void operator *= (const TYPE s) { SIMD::Multiply(this->Size(), s, this->Data()); }
  inline void operator *= (const AlignedVectorX<TYPE> &S) {
    SIMD::Multiply(this->Size(), S.Data(), this->Data());
  }
  inline void operator /= (const TYPE d) { SIMD::Multiply(this->Size(), 1 / d, this->Data()); }

  inline void Set(const TYPE v) { SIMD::Set(this->Size(), this->Data(), v); }
  inline void Set(const AlignedVectorX<TYPE> &V) { AlignedVector<TYPE>::Set(V); }
  inline void Set(const TYPE *V, const int N) { AlignedVector<TYPE>::Set(V, N); }
  inline void Set(const TYPE *V) { AlignedVector<TYPE>::Set(V); }

  inline void Get(TYPE *V) const { AlignedVector<TYPE>::Get(V); }
  inline void GetBlock(const int i, Vector2f &v) const;
  inline void GetBlock(const int i, Vector3f &v) const;
  inline void GetBlock(const int i, Vector6f &v) const;
  inline void GetBlock(const int i, Vector9f &v) const;
  inline void GetBlock(const int i, AlignedVector3f &v) const;
  inline void GetBlock(const int i, AlignedVector6f &v) const;
  inline void GetBlock(const int i, AlignedVector9f &v) const;
  inline void GetBlock(const int i, AlignedVectorX<float> &V) const;
  inline void SetBlock(const int i, const Vector2f &v);
  inline void SetBlock(const int i, const Vector3f &v);
  inline void SetBlock(const int i, const Vector6f &v);
  inline void SetBlock(const int i, const Vector9f &v);
  inline void SetBlock(const int i, const AlignedVector3f &v);
  inline void SetBlock(const int i, const AlignedVector6f &v);
  inline void SetBlock(const int i, const AlignedVector9f &v);
  inline void IncreaseBlock(const int i, const Vector2f &v);
  inline void IncreaseBlock(const int i, const AlignedVector3f &v);
  inline void IncreaseBlock(const int i, const Vector6f &v);

  inline TYPE SquaredLength() const { return SIMD::SquaredLength(this->Size(), this->Data()); }
  inline void MakeSquared() { SIMD::Square(this->Size(), this->Data()); }
  inline void MakeSquareRoot() { SIMD::SquareRoot(this->Size(), this->Data()); }
  inline void MakeMinus() { SIMD::Minus(this->Size(), this->Data()); }
  inline void MakeMinus(const int i) { SIMD::Minus(i, this->Size(), this->Data()); }
  inline void MakeInverse(const TYPE s = 1, const bool chkZero = false, const TYPE eps = 0) {
    SIMD::Inverse(this->Size(), this->Data(), s, chkZero, eps);
  }
  inline void GetInverse(AlignedVectorX<TYPE> &V, const TYPE s = 1, const bool chkZero = false,
                         const TYPE eps = 0) const {
    V = *this;
    V.MakeInverse(s, chkZero, eps);
  }
  inline TYPE Sum() const { return Sum(this->Size()); }
  inline TYPE Sum(const int N) const { return SIMD::Sum(N, this->Data()); }
  inline TYPE Sum(const int i1, const int i2) const { return SIMD::Sum(i1, i2, this->Data()); }
  inline TYPE Mean() const { return SIMD::Mean(this->Size(), this->Data()); }
  inline TYPE Variance() const { return SIMD::Variance(this->Size(), this->Data()); }
  inline TYPE Variance(const TYPE u) const {
    return SIMD::Variance(this->Size(), this->Data(), u);
  }
  inline TYPE Maximal() const { return SIMD::Maximal(this->Size(), this->Data()); }
  inline TYPE Minimal() const { return SIMD::Minimal(this->Size(), this->Data()); }
  inline TYPE Dot(const AlignedVectorX<TYPE> &V) const {
#ifdef CFG_DEBUG
    UT_ASSERT(V.Size() == this->Size());
#endif
    return Dot(V, V.Size());
  }
  inline TYPE Dot(const AlignedVectorX<TYPE> &V, const int N) const {
#ifdef CFG_DEBUG
    UT_ASSERT(N <= V.Size() && N <= this->Size());
#endif
    return SIMD::Dot<0>(N, this->Data(), V.Data());
  }
  inline void Scale(const AlignedVector<TYPE> &S) {
    const int N = this->Size();
#ifdef CFG_DEBUG
    UT_ASSERT(S.Size() == N);
#endif
    SIMD::Multiply(N, S.Data(), this->Data());
  }
  inline void GetScaled(const TYPE s, AlignedVectorX<TYPE> &V) const {
    const int N = this->Size();
    V.Resize(N);
    SIMD::Multiply(N, s, this->Data(), V.Data());
  }
  inline void GetScaled(const AlignedVector<TYPE> &S, AlignedVector<TYPE> &V) const {
    const int N = this->Size();
#ifdef CFG_DEBUG
    UT_ASSERT(S.Size() == N);
#endif
    V.Resize(N);
    SIMD::Multiply(N, S.Data(), this->Data(), V.Data());
  }

  inline void Random(const TYPE vMax) { UT::Random(this->Data(), this->Size(), -vMax, vMax); }

  inline void Print(const bool e = false) const {
    const AlignedVector<TYPE> &V = *this;
    const int N = V.Size();
    for (int i = 0; i < N; ++i) {
      if (e) {
        UT::Print("%e ", V[i]);
      } else {
        UT::Print("%f ", V[i]);
      }
    }
    UT::Print("\n");
  }
  inline void Save(const std::string fileName, const bool e = false) const {
    FILE *fp = fopen(fileName.c_str(), "w");
    const AlignedVector<TYPE> &V = *this;
    const int N = V.Size();
    for (int i = 0; i < N; ++i) {
      if (e) {
        fprintf(fp, "%e\n", V[i]);
      } else {
        fprintf(fp, "%f\n", V[i]);
      }
    }
    fclose(fp);
    UT::PrintSaved(fileName);
  }
  inline bool AssertEqual(const AlignedVectorX<TYPE> &V,
                          const int verbose = 1,
                          const std::string str = "",
                          const TYPE epsAbs = 0,
                          const TYPE epsRel = 0) const {
    const AlignedVector<TYPE> &_V = *this;
    const int N = _V.Size();
    bool equal = V.Size() == N;
    //const int verbose1 = verbose & 3, verbose2 = verbose >> 2;
    const int verbose1 = verbose, verbose2 = verbose;
    for (int i = 0; i < N && equal; ++i) {
      equal = UT::AssertEqual(_V[i], V[i], verbose1,
                              UT::String("%s[%d]", str.c_str(), i),
                              epsAbs, epsRel);
    }
    if (equal) {
      return true;
    } else if (verbose2) {
      UT::PrintSeparator();
      Print(verbose2 > 1);
      UT::PrintSeparator();
      V.Print(verbose2 > 1);
      const AlignedVectorX<TYPE> E = *this - V;
      UT::PrintSeparator();
      E.Print(verbose2 > 1);
    }
    return false;
  }
  inline bool AssertEqual(const std::string fileName,
                          const int verbose = 1,
                          const std::string str = "",
                          const TYPE epsAbs = 0,
                          const TYPE epsRel = 0) const {
    AlignedVectorX<TYPE> V;
    V.LoadB(fileName);
    return AssertEqual(V, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    bool zero = true;
    //const int verbose1 = verbose & 3, verbose2 = verbose >> 2;
    const int verbose1 = verbose, verbose2 = verbose;
    const AlignedVector<TYPE> &V = *this;
    const int N = V.Size();
    for (int i = 0; i < N && zero; ++i) {
      zero = UT::AssertZero(V[i], verbose1, UT::String("%s[%d]", str.c_str(), i),
                            epsAbs, epsRel);
    }
    if (zero) {
      return true;
    } else if (verbose2) {
      UT::PrintSeparator();
      Print(verbose2 > 1);
    }
    return false;
  }

  static inline void ApB(const AlignedVectorX<TYPE> &A, const AlignedVectorX<TYPE> &B,
                         AlignedVectorX<TYPE> &ApB) {
    const int N = A.Size();
#ifdef CFG_DEBUG
    UT_ASSERT(B.Size() == N);
#endif
    ApB.Resize(N);
    SIMD::Add(N, A.Data(), B.Data(), ApB.Data());
  }
  static inline void AmB(const AlignedVectorX<TYPE> &A, const AlignedVectorX<TYPE> &B,
                         AlignedVectorX<TYPE> &AmB) {
    const int N = A.Size();
#ifdef CFG_DEBUG
    UT_ASSERT(B.Size() == N);
#endif
    AmB.Resize(N);
    SIMD::Subtract(N, A.Data(), B.Data(), AmB.Data());
  }
  static inline void AmB(const AlignedVectorX<TYPE> &A, const TYPE B, AlignedVectorX<TYPE> &AmB) {
    const int N = A.Size();
    AmB.Resize(N);
    SIMD::Subtract(N, A.Data(), B, AmB.Data());
  }
  static inline void AB(const AlignedVectorX<TYPE> &A, const AlignedVectorX<TYPE> &B,
                        AlignedVectorX<TYPE> &AB) {
    const int N = A.Size();
#ifdef CFG_DEBUG
    UT_ASSERT(B.Size() == N);
#endif
    AB.Resize(N);
    SIMD::Multiply(N, A.Data(), B.Data(), AB.Data());
  }
  static inline void AddABTo(const AlignedVectorX<TYPE> &A, const AlignedVectorX<TYPE> &B,
                             AlignedVectorX<TYPE> &AB) {
    const int N = A.Size();
#ifdef CFG_DEBUG
    UT_ASSERT(B.Size() == N && AB.Size() == N);
#endif
    SIMD::MultiplyAddTo(N, A.Data(), B.Data(), AB.Data());
  }
};

template<> inline void AlignedVectorX<float>::GetBlock(const int i, Vector2f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 2 <= this->Size());
#endif
  memcpy(&v, this->Data() + i, sizeof(Vector2f));
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, Vector2f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 2 <= this->Size());
#endif
  v.v0() = static_cast<float>(m_data[i]);
  v.v1() = static_cast<float>(m_data[i + 1]);
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, Vector3f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  memcpy(&v, this->Data() + i, sizeof(Vector3f));
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, Vector3f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  v.v0() = static_cast<float>(m_data[i]);
  v.v1() = static_cast<float>(m_data[i + 1]);
  v.v2() = static_cast<float>(m_data[i + 2]);
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, Vector6f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  memcpy(&v, m_data + i, 24);
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, Vector6f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  for (int _i = 0; _i < 6; ++_i) {
    v[_i] = static_cast<float>(m_data[i + _i]);
  }
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, Vector9f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  memcpy(&v, m_data + i, 36);
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, Vector9f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  for (int _i = 0; _i < 9; ++_i) {
    v[_i] = static_cast<float>(m_data[i + _i]);
  }
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, AlignedVector3f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= m_N);
#endif
  memcpy(&v, m_data + i, 12);
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, AlignedVector3f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= m_N);
#endif
  v.v0() = static_cast<float>(m_data[i]);
  v.v1() = static_cast<float>(m_data[i + 1]);
  v.v2() = static_cast<float>(m_data[i + 2]);
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, AlignedVector6f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  memcpy(&v, m_data + i, 24);
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, AlignedVector6f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  for (int _i = 0; _i < 6; ++_i) {
    v[_i] = static_cast<float>(m_data[i + _i]);
  }
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, AlignedVector9f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  memcpy(&v, m_data + i, 36);
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, AlignedVector9f &v) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  for (int _i = 0; _i < 9; ++_i) {
    v[_i] = static_cast<float>(m_data[i + _i]);
  }
}
template<> inline void AlignedVectorX<float>::GetBlock(const int i, AlignedVectorX<float> &V) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + V.Size() <= this->Size());
#endif
  V.Set(this->Data() + i, V.Size());
}
template<> inline void AlignedVectorX<double>::GetBlock(const int i, AlignedVectorX<float> &V) const {
  const int N = V.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + N <= this->Size());
#endif
  for (int i = 0; i < N; ++i) {
    V[i] = static_cast<float>(m_data[i]);
  }
}

template<> inline void AlignedVectorX<float>::SetBlock(const int i, const Vector2f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 2 <= this->Size());
#endif
  memcpy(this->Data() + i, &v, sizeof(Vector2f));
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const Vector2f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 2 <= this->Size());
#endif
  m_data[i] = static_cast<double>(v.v0());
  m_data[i + 1] = static_cast<double>(v.v1());
}
template<> inline void AlignedVectorX<float>::SetBlock(const int i, const Vector3f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  memcpy(this->Data() + i, &v, sizeof(Vector3f));
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const Vector3f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  m_data[i] = static_cast<double>(v.v0());
  m_data[i + 1] = static_cast<double>(v.v1());
  m_data[i + 2] = static_cast<double>(v.v2());
}
template<> inline void AlignedVectorX<float>::SetBlock(const int i, const Vector6f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  memcpy(m_data + i, &v, 24);
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const Vector6f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  for (int _i = 0; _i < 6; ++_i) {
    m_data[i + _i] = static_cast<double>(v[_i]);
  }
}
template<> inline void AlignedVectorX<float>::SetBlock(const int i, const Vector9f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  memcpy(m_data + i, &v, 36);
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const Vector9f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  for (int _i = 0; _i < 9; ++_i) {
    m_data[i + _i] = static_cast<double>(v[_i]);
  }
}
template<> inline void AlignedVectorX<float>::SetBlock(const int i, const AlignedVector3f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  memcpy(this->Data() + i, &v, sizeof(Vector3f));
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const AlignedVector3f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  m_data[i] = static_cast<double>(v.v0());
  m_data[i + 1] = static_cast<double>(v.v1());
  m_data[i + 2] = static_cast<double>(v.v2());
}
template<> inline void AlignedVectorX<float>::SetBlock(const int i, const AlignedVector6f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  memcpy(m_data + i, &v, 24);
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const AlignedVector6f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= m_N);
#endif
  for (int _i = 0; _i < 6; ++_i) {
    m_data[i + _i] = static_cast<double>(v[_i]);
  }
}
template<> inline void AlignedVectorX<float>::SetBlock(const int i, const AlignedVector9f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  memcpy(m_data + i, &v, 36);
}
template<> inline void AlignedVectorX<double>::SetBlock(const int i, const AlignedVector9f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 9 <= m_N);
#endif
  for (int _i = 0; _i < 9; ++_i) {
    m_data[i + _i] = static_cast<double>(v[_i]);
  }
}

template<> inline void AlignedVectorX<float>::IncreaseBlock(const int i, const Vector2f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 2 <= this->Size());
#endif
  m_data[i] += v.v0();
  m_data[i + 1] += v.v1();
}
template<> inline void AlignedVectorX<double>::IncreaseBlock(const int i, const Vector2f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 2 <= this->Size());
#endif
  m_data[i] += static_cast<double>(v.v0());
  m_data[i + 1] += static_cast<double>(v.v1());
}
template<> inline void AlignedVectorX<float>::IncreaseBlock(const int i, const AlignedVector3f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  m_data[i] += v.v0();
  m_data[i + 1] += v.v1();
  m_data[i + 2] += v.v2();
}
template<> inline void AlignedVectorX<double>::IncreaseBlock(const int i, const AlignedVector3f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= this->Size());
#endif
  m_data[i] += static_cast<double>(v.v0());
  m_data[i + 1] += static_cast<double>(v.v1());
  m_data[i + 2] += static_cast<double>(v.v2());
}
template<> inline void AlignedVectorX<float>::IncreaseBlock(const int i, const Vector6f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= this->Size());
#endif
  for (int _i = 0; _i < 6; ++_i) {
    m_data[i + _i] += v[_i];
  }
}
template<> inline void AlignedVectorX<double>::IncreaseBlock(const int i, const Vector6f &v) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 6 <= this->Size());
#endif
  for (int _i = 0; _i < 6; ++_i) {
    m_data[i + _i] += v[_i];
  }
}

class AlignedVectorXf : public AlignedVectorX<float> {
 public:
  inline AlignedVectorXf() : AlignedVectorX<float>() {}
  inline AlignedVectorXf(void *V, const int N, const bool own = true) :
                         AlignedVectorX<float>(V, N, own) {}
  inline AlignedVectorXf(const AlignedVectorX<float> &V) { *this = V; }
  //inline void operator = (const AlignedVectorX<double> &V) {
  //  const int N = V.Size();
  //  Resize(N);
  //  SIMD::Convert<double, float>(N, V.Data(), this->Data());
  //}
  inline void operator *= (const float s) { SIMD::Multiply(Size(), s, Data()); }
  inline void operator *= (const xp128f &s) { SIMD::Multiply<0>(Size(), s, Data()); }
  inline void operator *= (const AlignedVectorXf &S) {
    SIMD::Multiply(this->Size(), S.Data(), this->Data());
  }
  inline void Bind(void *V, const int N) {
    AlignedVectorX<float>::Bind(V, N);
    m_capacity = SIMD_FLOAT_CEIL(N);
  }
  inline int BindSize(const int N) const {
    const int NC = SIMD_FLOAT_CEIL(N);
    return AlignedVectorX<float>::BindSize(NC);
  }
  inline void Get(AlignedVectorXf &V) const {
    V.Resize(Size());
    AlignedVectorX<float>::Get(V.Data());
  }
  inline void GetHistogram(AlignedVector<int> &H, AlignedVectorXf &P, const int Nb,
                           const float vMin = FLT_MAX, const float vMax = FLT_MAX) const {
    P.Set(*this);
    const float _vMin = vMin == FLT_MAX ? P.Minimal() : vMin;
    const float _vMax = vMax == FLT_MAX ? P.Maximal() : vMax;
    P -= _vMin;
    P *= (Nb - 1) / (_vMax - _vMin);
    int *ibs = (int *) P.Data();
    const int N = P.Size();
    SIMD::Convert(N, P.Data(), ibs);

    H.Resize(Nb);
    H.MakeZero();
    for (int i = 0; i < N; ++i) {
      const int ib = ibs[i];
      if (ib >= 0 && ib < Nb) {
        ++H[ib];
      }
    }
    P.Resize(Nb);
    SIMD::Convert<int, float>(Nb, H.Data(), P.Data());
    const float _s = P.Sum();
    P /= _s;
  }
};

class AlignedVectorXd : public AlignedVectorX<double> {
 public:
  inline AlignedVectorXd() : AlignedVectorX<double>() {}
  inline AlignedVectorXd(void *V, const int N, const bool own = true) :
                         AlignedVectorX<double>(V, N, own) {}
  inline AlignedVectorXd(const AlignedVectorX<double> &V) { *this = V; }
  inline void operator = (const AlignedVectorX<float> &V) {
    const int N = V.Size();
    Resize(N);
    SIMD::Convert<float, double>(N, V.Data(), this->Data());
  }
  inline void Set(const float *V) {
    for (int i = 0; i < m_N; ++i) {
      m_data[i] = static_cast<double>(V[i]);
    }
  }
  inline void Set(const AlignedVectorXf &V) {
    Resize(V.Size());
    Set(V.Data());
  }
  inline void Get(AlignedVectorXf &V) const {
    const int N = Size();
    V.Resize(N);
    for (int i = 0; i < N; ++i) {
      V[i] = static_cast<float>(m_data[i]);
    }
  }
};
}

#ifdef CFG_DEBUG_EIGEN
template<int N>
class EigenVectorNf : public Eigen::Matrix<float, N, 1> {
 public:
  inline EigenVectorNf() : Eigen::Matrix<float, N, 1>() {}
  inline EigenVectorNf(const Eigen::Matrix<float, N, 1> &e_v) : Eigen::Matrix<float, N, 1>(e_v) {}
  inline EigenVectorNf(const LA::AlignedVectorNf<N> &v) : Eigen::Matrix<float, N, 1>() {
    EigenVectorNf<N> &e_v = *this;
    for (int i = 0; i < N; ++i) {
      e_v(i, 0) = v[i];
    }
  }
  inline void operator = (const Eigen::Matrix<float, N, 1> &e_v) { *((Eigen::Matrix<float, N, 1> *) this) = e_v; }
  inline const float& operator[] (const int i) const { return (*this)(i, 0); }
  inline       float& operator[] (const int i)       { return (*this)(i, 0); }
  inline LA::AlignedVectorNf<N> GetAlignedVectorNf() const {
    LA::AlignedVectorNf<N> v;
    const EigenVectorNf<N> &e_v = *this;
    for (int i = 0; i < N; ++i) {
      v[i] = e_v[i];
    }
    return v;
  }
  inline void Print(const bool e = false) const { GetAlignedVectorNf().Print(e); }
  inline bool AssertEqual(const LA::AlignedVectorNf<N> &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVectorNf().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVectorNf &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVectorNf(), verbose, str, epsAbs, epsRel);
  }
};
class EigenVector12f : public EigenVectorNf<12> {
 public:
  inline EigenVector12f() : EigenVectorNf<12>() {}
  inline EigenVector12f(const EigenVectorNf<12> &e_v) : EigenVectorNf<12>(e_v) {}
  inline EigenVector12f(const Eigen::Matrix<float, 12, 1> &e_v) : EigenVectorNf<12>(e_v) {}
  inline EigenVector12f(const LA::AlignedVectorNf<12> &v) : EigenVectorNf<12>(v) {}
};
class EigenVector13f : public EigenVectorNf<13> {
 public:
  inline EigenVector13f() : EigenVectorNf<13>() {}
  inline EigenVector13f(const EigenVectorNf<13> &e_v) : EigenVectorNf<13>(e_v) {}
  inline EigenVector13f(const Eigen::Matrix<float, 13, 1> &e_v) : EigenVectorNf<13>(e_v) {}
  inline EigenVector13f(const LA::AlignedVectorNf<13> &v) : EigenVectorNf<13>(v) {}
  inline EigenVector13f(const float &e_v0, const EigenVector6f &e_v1, const EigenVector6f &e_v2) {
    EigenVector13f &v = *this;
    v[0] = e_v0;
    v.block<6, 1>(1, 0) = e_v1;
    v.block<6, 1>(7, 0) = e_v2;
  }
};
class EigenVector14f : public EigenVectorNf<14> {
 public:
  inline EigenVector14f() : EigenVectorNf<14>() {}
  inline EigenVector14f(const EigenVectorNf<14> &e_v) : EigenVectorNf<14>(e_v) {}
  inline EigenVector14f(const Eigen::Matrix<float, 14, 1> &e_v) : EigenVectorNf<14>(e_v) {}
  inline EigenVector14f(const LA::AlignedVectorNf<14> &v) : EigenVectorNf<14>(v) {}
  inline EigenVector14f(const EigenVector13f &e_v0, const float &e_v1) {
    EigenVector14f &v = *this;
    v.block<13, 1>(0, 0) = e_v0;
    v[13] = e_v1;
  }
};
class EigenVector15f : public EigenVectorNf<15> {
 public:
  inline EigenVector15f() : EigenVectorNf<15>() {}
  inline EigenVector15f(const EigenVectorNf<15> &e_v) : EigenVectorNf<15>(e_v) {}
  inline EigenVector15f(const Eigen::Matrix<float, 15, 1> &e_v) : EigenVectorNf<15>(e_v) {}
};
class EigenVector30f : public EigenVectorNf<30> {
 public:
  inline EigenVector30f() : EigenVectorNf<30>() {}
  inline EigenVector30f(const EigenVectorNf<30> &e_v) : EigenVectorNf<30>(e_v) {}
  inline EigenVector30f(const Eigen::Matrix<float, 30, 1> &e_v) : EigenVectorNf<30>(e_v) {}
  inline EigenVector30f(const LA::AlignedVectorNf<30> &v) : EigenVectorNf<30>(v) {}
  inline EigenVector30f(const EigenVector6f &e_v0, const EigenVector9f &e_v1,
                        const EigenVector6f &e_v2, const EigenVector9f &e_v3) {
    EigenVector30f &v = *this;
    v.block<6, 1>(0, 0) = e_v0;
    v.block<9, 1>(6, 0) = e_v1;
    v.block<6, 1>(15, 0) = e_v2;
    v.block<9, 1>(21, 0) = e_v3;
  }
};

template<typename TYPE>
class EigenVectorX : public Eigen::Matrix<TYPE, Eigen::Dynamic, 1> {
 public:
  inline EigenVectorX() : Eigen::Matrix<TYPE, Eigen::Dynamic, 1>() {}
  inline EigenVectorX(const Eigen::Matrix<TYPE, Eigen::Dynamic, 1> &e_V) :
    Eigen::Matrix<TYPE, Eigen::Dynamic, 1>(e_V) {}
  inline EigenVectorX(const AlignedVector<TYPE> &V) : Eigen::Matrix<TYPE, Eigen::Dynamic, 1>() {
    const int N = V.Size();
    Resize(N);
    EigenVectorX<TYPE> &e_V = *this;
    for (int i = 0; i < N; ++i) {
      e_V[i] = V[i];
    }
  }
  inline void operator = (const Eigen::Matrix<TYPE, Eigen::Dynamic, 1> &e_V) {
    *((Eigen::Matrix<TYPE, Eigen::Dynamic, 1> *) this) = e_V;
  }
  inline const TYPE& operator[] (const int i) const { return (*this)(i, 0); }
  inline       TYPE& operator[] (const int i)       { return (*this)(i, 0); }
  inline int Size() const { return int(this->rows()); }
  inline void Resize(const int N) { this->resize(N); }
  inline void MakeZero() { this->setZero(); }
  inline void Insert(const int i, const EigenVectorX<TYPE> &e_Vi) {
    EigenVectorX<TYPE> e_V;
    const int N = e_Vi.Size(), N1 = Size(), N2 = N1 + N, N3 = N1 - i;
    e_V.Resize(N2);
    e_V.block(0, 0, i, 1) = this->block(0, 0, i, 1);
    e_V.block(i, 0, N, 1) = e_Vi;
    e_V.block(i + N, 0, N3, 1) = this->block(i, 0, N3, 1);
    this->swap(e_V);
  }
  inline void PushZero(const int N) {
    const int _N = Size();
    EigenVectorX<TYPE> e_V;
    e_V.Resize(_N + N);
    e_V.MakeZero();
    e_V.block(0, 0, _N, 1) = *this;
    *this = e_V;
  }
  inline void InsertZero(const int i, const int N) {
    EigenVectorX<TYPE> e_V;
    const int N1 = Size(), N2 = N1 + N, N3 = N1 - i;
    e_V.Resize(N2);
    e_V.block(0, 0, i, 1) = this->block(0, 0, i, 1);
    e_V.block(i, 0, N, 1).setZero();
    e_V.block(i + N, 0, N3, 1) = this->block(i, 0, N3, 1);
    this->swap(e_V);
  }
  inline void Erase(const int i, const int N) {
    EigenVectorX<TYPE> e_V;
    const int N1 = Size(), N2 = N1 - N, N3 = N2 - i;
    e_V.Resize(N2);
    e_V.block(0, 0, i, 1) = this->block(0, 0, i, 1);
    e_V.block(i, 0, N3, 1) = this->block(i + N, 0, N3, 1);
    this->swap(e_V);
  }
  inline LA::AlignedVectorXf GetAlignedVectorXf() const {
    LA::AlignedVectorXf V;
    const int N = Size();
    V.Resize(N);
    const EigenVectorX<TYPE> &e_V = *this;
    for (int i = 0; i < N; ++i) {
      V[i] = e_V[i];
    }
    return V;
  }
  inline LA::AlignedVectorXd GetAlignedVectorXd() const {
    LA::AlignedVectorXd V;
    const int N = Size();
    V.Resize(N);
    const EigenVectorX<TYPE> &e_V = *this;
    for (int i = 0; i < N; ++i) {
      V[i] = e_V[i];
    }
    return V;
  }
  inline void Print(const bool e = false) const { GetAlignedVectorXf().Print(e); }
  inline bool AssertEqual(const LA::AlignedVectorXf &V,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVectorXf().AssertEqual(V, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::AlignedVectorXd &V,
                          const int verbose = 1, const std::string str = "",
                          const double epsAbs = 0.0, const double epsRel = 0.0) const {
    return GetAlignedVectorXd().AssertEqual(V, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVectorX<TYPE> &e_V, 
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_V.GetAlignedVectorXf(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVectorXf().AssertZero(verbose, str, epsAbs, epsRel);
  }
};
typedef EigenVectorX<float> EigenVectorXf;
typedef EigenVectorX<double> EigenVectorXd;
#endif
#endif
