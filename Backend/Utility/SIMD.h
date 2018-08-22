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
#ifndef _UTILITY_SIMD_H_
#define _UTILITY_SIMD_H_

#ifdef __SSE__
#include <xmmintrin.h>
#endif // __SSE__

#ifdef __SSE2__
#include <emmintrin.h>
#endif // __SSE2__

#ifdef __SSE4_1__
#include <smmintrin.h>
#endif // __SSE4_1__

#include <string.h>     // for memcpy

#include <climits>      // for USHRT_MAX

#include "simd_sse_neon.h"

// TODO (yanghongtian) : NEON implemetation.

#define SIMD_DOUBLE_FLOOR(N) ((N) - ((N) & 1))
#define SIMD_DOUBLE_CEIL(N) (((N) + 1) & (~1))
#define SIMD_FLOAT_FLOOR(N) ((N) - ((N) & 3))
#define SIMD_FLOAT_CEIL(N) (((N) + 3) & (~3))
#define SIMD_SHORT_FLOOR(N) ((N) - ((N) & 7))
#define SIMD_SHORT_CEIL(N) (((N) + 7) & (~7))
#define SIMD_BYTE_FLOOR(N) ((N) - ((N) & 15))
#define SIMD_BYTE_CEIL(N) (((N) + 15) & (~15))

// [NOTE] Cross-platform issues:
// 1. _mm_malloc & _mm_free_ are the cross platform version for
//   _aligned_malloc & _aligned_free (WIN32) or
//   memalign & free (POSIX)

namespace SIMD {

template<class TYPE> inline TYPE* Malloc(const int N = 1) {
//#ifdef __IBA_SSE__
#ifdef _MSC_VER
  return (TYPE*) _mm_malloc(N * sizeof(TYPE), SIMD_ALIGN);
#else
  void* ptr = NULL;
  if (0 != posix_memalign(&ptr, SIMD_ALIGN, N * sizeof(TYPE))) {
    return reinterpret_cast<TYPE*>(NULL);
  }
  return reinterpret_cast<TYPE*>(ptr);
#endif
}
template<class TYPE> inline void Free(TYPE* p) {
//#ifdef __IBA_SSE__
#ifdef _MSC_VER
  _mm_free(p);
#else
  free(p);
#endif
}
template<class TYPE> inline int Ceil(const int N) { return N; }
template<> inline int Ceil<float>(const int N) { return SIMD_FLOAT_CEIL(N); }
template<> inline int Ceil<double>(const int N) { return SIMD_DOUBLE_CEIL(N); }

template<class T>
class aligned_allocator
{
public:
  typedef size_t    size_type;
  typedef std::ptrdiff_t difference_type;
  typedef T*        pointer;
  typedef const T*  const_pointer;
  typedef T&        reference;
  typedef const T&  const_reference;
  typedef T         value_type;

  template<class U>
  struct rebind
  {
    typedef aligned_allocator<U> other;
  };

  pointer address( reference value ) const
  {
    return &value;
  }

  const_pointer address( const_reference value ) const
  {
    return &value;
  }

  aligned_allocator()
  {
  }

  aligned_allocator( const aligned_allocator& )
  {
  }

  template<class U>
  aligned_allocator( const aligned_allocator<U>& )
  {
  }

  ~aligned_allocator()
  {
  }

  size_type max_size() const
  {
    return (std::numeric_limits<size_type>::max)();
  }

  pointer allocate( size_type num, const void* hint = 0 )
  {
    return Malloc<T>(num);
  }

  void construct( pointer p, const T& value )
  {
    ::new( p ) T( value );
  }

  void destroy( pointer p )
  {
    p->~T();
  }

  void deallocate( pointer p, size_type /*num*/ )
  {
    Free(p);
  }

  bool operator!=(const aligned_allocator<T>& ) const
  { return false; }

  bool operator==(const aligned_allocator<T>& ) const
  { return true; }
};

template <class T>
class vector : public std::vector<T, aligned_allocator<T> > {
};

inline void Cross012(const xp128f &u1, const xp128f &u2, xp128f &u1xu2) {
  // TODO (yanghongtian): Rooms for optimization...
  u1xu2[0] = u1[1] * u2[2] - u1[2] * u2[1];
  u1xu2[1] = u1[2] * u2[0] - u1[0] * u2[2];
  u1xu2[2] = u1[0] * u2[1] - u1[1] * u2[0];
}

template<typename TYPE> inline void Set(const int N, TYPE *Vs, const TYPE v);
template<> inline void Set<float>(const int N, float *V, const float v) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f v1 = xp128f::get(v);
  xp128f *v2 = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v2) {
    *v2 = v1;
  }
  for (int i = NF; i < N; ++i) {
    V[i] = v;
  }
}

template<typename TYPE_SRC, typename TYPE_DST> inline void Convert(const int N, const TYPE_SRC *Vs,
                                                                   TYPE_DST *Vd);
template<> inline void Convert<int, float>(const int N, const int *Vs, float *Vd) {
  for (int i = 0; i < N; ++i) {
    Vd[i] = float(Vs[i]);
  }
}
template<> inline void Convert<float, float>(const int N, const float *Vs, float *Vd) {
  memcpy(Vd, Vs, sizeof(float) * N);
}
template<> inline void Convert<float, int>(const int N, const float *Vs, int *Vd) {
  // TODO (yanghongtian) : SSE + NEON implemetation
  for (int i = 0; i < N; ++i) {
    Vd[i] = int(Vs[i]);
  }
}
template<> inline void Convert<float, double>(const int N, const float *Vs, double *Vd) {
  for (int i = 0; i < N; ++i) {
    Vd[i] = double(Vs[i]);
  }
}
template<> inline void Convert<double, float>(const int N, const double *Vs, float *Vd) {
  for (int i = 0; i < N; ++i) {
    Vd[i] = float(Vs[i]);
  }
}
template<> inline void Convert<double, double>(const int N, const double *Vs, double *Vd) {
  memcpy(Vd, Vs, sizeof(double) * N);
}

template<typename TYPE> inline void Add(const int N, const TYPE *Va, const TYPE *Vb, TYPE *Vapb);
template<> inline void Add<float>(const int N, const float *Va, const float *Vb, float *Vapb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (xp128f *) Va, *b = (xp128f *) Vb;
  xp128f *apb = (xp128f *) Vapb;
  for (int i = 0; i < NF; i += 4, ++a, ++b, ++apb) {
    *apb = *a + *b;
  }
  for (int i = NF; i < N; ++i) {
    Vapb[i] = Va[i] + Vb[i];
  }
}
template<typename TYPE> inline void Add(const int N, const TYPE *Va, TYPE *Vb);
template<> inline void Add<float>(const int N, const float *Va, float *Vb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (xp128f *) Va;
  xp128f *b = (xp128f *) Vb;
  for (int i = 0; i < NF; i += 4, ++a, ++b) {
    *b += *a;
  }
  for (int i = NF; i < N; ++i) {
    Vb[i] = Va[i] + Vb[i];
  }
}
template<> inline void Add<double>(const int N, const double *Va, double *Vb) {
  for (int i = 0; i < N; ++i) {
    Vb[i] = Va[i] + Vb[i];
  }
}
template<typename TYPE> inline void Add(const int N, const TYPE a, TYPE *Vb);
template<> inline void Add<float>(const int N, const float a, float *Vb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f _a = xp128f::get(a);
  xp128f *b = (xp128f *) Vb;
  for (int i = 0; i < NF; i += 4, ++b) {
    *b += _a;
  }
  for (int i = NF; i < N; ++i) {
    Vb[i] = a + Vb[i];
  }
}

template<typename TYPE> inline void Subtract(const int N, const TYPE *Va, const TYPE *Vb,
                                             TYPE *Vamb);
template<> inline void Subtract<float>(const int N, const float *Va, const float *Vb, float *Vamb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (xp128f *) Va, *b = (xp128f *) Vb;
  xp128f *amb = (xp128f *) Vamb;
  for (int i = 0; i < NF; i += 4, ++a, ++b, ++amb) {
    *amb = *a - *b;
  }
  for (int i = NF; i < N; ++i) {
    Vamb[i] = Va[i] - Vb[i];
  }
}
template<> inline void Subtract<double>(const int N, const double *Va, const double *Vb,
                                        double *Vamb) {
  for (int i = 0; i < N; ++i) {
    Vamb[i] = Va[i] - Vb[i];
  }
}
template<typename TYPE> inline void Subtract(const int N, const TYPE *Va, const TYPE b, TYPE *Vamb);
template<> inline void Subtract<float>(const int N, const float *Va, const float b, float *Vamb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (xp128f *) Va;
  const xp128f _b = xp128f::get(b);
  xp128f *amb = (xp128f *) Vamb;
  for (int i = 0; i < NF; i += 4, ++a, ++amb) {
    *amb = *a - _b;
  }
  for (int i = NF; i < N; ++i) {
    Vamb[i] = Va[i] - b;
  }
}
template<> inline void Subtract<double>(const int N, const double *Va, const double b,
                                        double *Vamb) {
  for (int i = 0; i < N; ++i) {
    Vamb[i] = Va[i] - b;
  }
}
template<typename TYPE> inline void Subtract(const int N, const TYPE *Va, TYPE *Vb);
template<> inline void Subtract<float>(const int N, const float *Va, float *Vb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (xp128f *) Va;
  xp128f *b = (xp128f *) Vb;
  for (int i = 0; i < NF; i += 4, ++a, ++b) {
    *b = *a - *b;
  }
  for (int i = NF; i < N; ++i) {
    Vb[i] = Va[i] - Vb[i];
  }
}
template<> inline void Subtract<double>(const int N, const double *Va, double *Vb) {
  for (int i = 0; i < N; ++i) {
    Vb[i] = Va[i] - Vb[i];
  }
}
template<typename TYPE> inline void Subtract(const int N, TYPE *Va, const TYPE *Vb);
template<> inline void Subtract<float>(const int N, float *Va, const float *Vb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f *a = (xp128f *) Va;
  const xp128f *b = (xp128f *) Vb;
  for (int i = 0; i < NF; i += 4, ++a, ++b) {
    *a -= *b;
  }
  for (int i = NF; i < N; ++i) {
    Va[i] = Va[i] - Vb[i];
  }
}
template<> inline void Subtract<double>(const int N, double *Va, const double *Vb) {
  for (int i = 0; i < N; ++i) {
    Va[i] = Va[i] - Vb[i];
  }
}

template<int SKIP> inline void Multiply(const int N, const xp128f &a, float *Vb);
template<> inline void Multiply<0>(const int N, const xp128f &a, float *Vb) {
  xp128f *b = (xp128f *) Vb;
  const int NF = SIMD_FLOAT_FLOOR(N);
  for (int i = 0; i < NF; i += 4, ++b) {
    *b *= a;
  }
  for (int i = NF; i < N; ++i) {
    Vb[i] *= a[0];
  }
}
template<> inline void Multiply<1>(const int N, const xp128f &a, float *Vb) {
  if (N > 4) {
    Vb[1] *= a[0];
    Vb[2] *= a[0];
    Vb[3] *= a[0];
    Multiply<0>(N - 4, a, Vb + 4);
  } else {
    for (int i = 1; i < N; ++i) {
      Vb[i] *= a[0];
    }
  }
}
template<> inline void Multiply<2>(const int N, const xp128f &a, float *Vb) {
  if (N > 4) {
    Vb[2] *= a[0];
    Vb[3] *= a[0];
    Multiply<0>(N - 4, a, Vb + 4);
  } else {
    for (int i = 2; i < N; ++i) {
      Vb[i] *= a[0];
    }
  }
}
template<> inline void Multiply<3>(const int N, const xp128f &a, float *Vb) {
  if (N > 4) {
    Vb[3] *= a[0];
    Multiply<0>(N - 4, a, Vb + 4);
  } else if (N == 4) {
    Vb[3] *= a[0];
  }
}
inline void Multiply(const int i, const int N, const xp128f &a, float *Vb) {
  switch (i) {
  case 0: Multiply<0>(N, a, Vb);      break;
  case 1: Multiply<1>(N, a, Vb);      break;
  case 2: Multiply<2>(N, a, Vb);      break;
  case 3: Multiply<3>(N, a, Vb);      break;
  case 4: Multiply<0>(N - 4, a, Vb + 4);  break;
  case 5: Multiply<1>(N - 4, a, Vb + 4);  break;
  case 6: Multiply<2>(N - 4, a, Vb + 4);  break;
  default: {
    const int NF = SIMD_FLOAT_FLOOR(i);
    Multiply(i - NF, N - NF, a, Vb + NF);
    break;
  }
  }
}
inline void Multiply(const int i, const int N, const float a, float *Vb) {
  const xp128f _a = xp128f::get(a);
  Multiply(i, N, _a, Vb);
}

template<typename TYPE> inline void Multiply(const int N, const TYPE *Va, const TYPE *Vb,
                                             TYPE *Vab);
template<> inline void Multiply<float>(const int N, const float *Va, const float *Vb, float *Vab) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (const xp128f *) Va, *b = (xp128f *) Vb;
  xp128f *ab = (xp128f *) Vab;
  for (int i = 0; i < NF; i += 4, ++a, ++b, ++ab) {
    * ab = (*a) * (*b);
  }
  for (int i = NF; i < N; ++i) {
    Vab[i] = Va[i] * Vb[i];
  }
}
template<> inline void Multiply<double>(const int N, const double *Va, const double *Vb, double *Vab) {
  for (int i = 0; i < N; ++i) {
    Vab[i] = Va[i] * Vb[i];
  }
}

template<typename TYPE> inline void Multiply(const int N, const TYPE a, TYPE *Vb);
template<> inline void Multiply<float>(const int N, const float a, float *Vb) {
  if (a == 1.0f) {
    return;
  }
  const xp128f _a = xp128f::get(a);
  Multiply<0>(N, _a, Vb);
}
template<> inline void Multiply<double>(const int N, const double a, double *Vb) {
  if (a == 1.0) {
    return;
  }
  for (int i = 0; i < N; ++i) {
    Vb[i] *= a;
  }
}

template<typename TYPE> inline void Multiply(const int N, const TYPE *Va, TYPE *Vb);
template<> inline void Multiply<float>(const int N, const float *Va, float *Vb) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (const xp128f *) Va;
  xp128f *b = (xp128f *) Vb;
  for (int i = 0; i < NF; i += 4, ++a, ++b) {
    (*b) *= (*a);
  }
  for (int i = NF; i < N; ++i) {
    Vb[i] *= Va[i];
  }
}

template<typename TYPE_A, typename TYPE_B> inline void Multiply(const int N, const TYPE_A a, const TYPE_B *Vb, TYPE_A *Vab);
inline void Multiply(const int N, const xp128f &a, const float *Vb, float *Vab) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *b = (xp128f *) Vb;
  xp128f *ab = (xp128f *) Vab;
  for (int i = 0; i < NF; i += 4, ++b, ++ab) {
    *ab = a * (*b);
  }
  for (int i = NF; i < N; ++i) {
    Vab[i] = a[0] * Vb[i];
  }
}
template<> inline void Multiply<float, float>(const int N, const float a, const float *Vb,
                                              float *Vab) {
  if (a == 1.0f) {
    Convert(N, Vb, Vab);
    return;
  }
  const xp128f _a = xp128f::get(a);
  Multiply(N, _a, Vb, Vab);
}
template<> inline void Multiply<double, double>(const int N, const double a, const double *Vb,
                                                double *Vab) {
  if (a == 1.0) {
    Convert(N, Vb, Vab);
    return;
  }
  for (int i = 0; i < N; ++i) {
    Vab[i] = a * Vb[i];
  }
}

template<int SKIP> inline void MultiplyAddTo(const int N, const xp128f &a, const float *Vb,
                                             float *Vab);
template<> inline void MultiplyAddTo<0>(const int N, const xp128f &a, const float *Vb, float *Vab) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *b = (xp128f *) Vb;
  xp128f *ab = (xp128f *) Vab;
  for (int i = 0; i < NF; i += 4, ++b, ++ab) {
    *ab += *b * a;
  }
  for (int i = NF; i < N; ++i) {
    Vab[i] += a[0] * Vb[i];
  }
}

template<> inline void MultiplyAddTo<1>(const int N, const xp128f &a, const float *Vb, float *Vab) {
  if (N > 4) {
    Vab[1] += a[0] * Vb[1];
    Vab[2] += a[0] * Vb[2];
    Vab[3] += a[0] * Vb[3];
    MultiplyAddTo<0>(N - 4, a, Vb + 4, Vab + 4);
  } else {
    for (int i = 1; i < N; ++i) {
      Vab[i] += a[0] * Vb[i];
    }
  }
}

template<> inline void MultiplyAddTo<2>(const int N, const xp128f &a, const float *Vb, float *Vab) {
  if (N > 4) {
    Vab[2] += a[0] * Vb[2];
    Vab[3] += a[0] * Vb[3];
    MultiplyAddTo<0>(N - 4, a, Vb + 4, Vab + 4);
  } else {
    for (int i = 2; i < N; ++i) {
      Vab[i] += a[0] * Vb[i];
    }
  }
}

template<> inline void MultiplyAddTo<3>(const int N, const xp128f &a, const float *Vb, float *Vab) {
  if (N > 4) {
    Vab[3] += a[0] * Vb[3];
    MultiplyAddTo<0>(N - 4, a, Vb + 4, Vab + 4);
  } else if (N == 4) {
    Vab[3] += a[0] * Vb[3];
  }
}

inline void MultiplyAddTo(const int i, const int N, const xp128f &a, const float *Vb, float *Vab) {
  switch (i) {
  case 0: MultiplyAddTo<0>(N, a, Vb, Vab);        break;
  case 1: MultiplyAddTo<1>(N, a, Vb, Vab);        break;
  case 2: MultiplyAddTo<2>(N, a, Vb, Vab);        break;
  case 3: MultiplyAddTo<3>(N, a, Vb, Vab);        break;
  case 4: MultiplyAddTo<0>(N - 4, a, Vb + 4, Vab + 4);  break;
  case 5: MultiplyAddTo<1>(N - 4, a, Vb + 4, Vab + 4);  break;
  case 6: MultiplyAddTo<2>(N - 4, a, Vb + 4, Vab + 4);  break;
  default: {
    const int NF = SIMD_FLOAT_FLOOR(i);
    MultiplyAddTo(i - NF, N - NF, a, Vb + NF, Vab + NF);
    break;
  }
  }
}

inline void MultiplyAddTo(const int N, const float a, const float *Vb, float *Vab) {
  const xp128f _a = xp128f::get(a);
  MultiplyAddTo(0, N, _a, Vb, Vab);
}
inline void MultiplyAddTo(const int i, const int N, const float a, const float *Vb, float *Vab) {
  const xp128f _a = xp128f::get(a);
  MultiplyAddTo(i, N, _a, Vb, Vab);
}

template<typename TYPE> inline void MultiplyAddTo(const int N, const TYPE *Va, const TYPE *Vb,
                                                  TYPE *Vab);
template<> inline void MultiplyAddTo<float>(const int N, const float *Va, const float *Vb,
                                            float *Vab) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  const xp128f *a = (const xp128f *) Va, *b = (xp128f *) Vb;
  xp128f *ab = (xp128f *) Vab;
  for (int i = 0; i < NF; i += 4, ++a, ++b, ++ab) {
    *ab += (*a) * (*b);
  }
  for (int i = NF; i < N; ++i) {
    Vab[i] = Va[i] * Vb[i] + Vab[i];
  }
}

template<int SKIP> inline void Minus(const int N, float *V);
template<> inline void Minus<0>(const int N, float *V) {
  const xp128f zero = xp128f::get(0.f);
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f *v = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v) {
    *v = zero - *v;
  }
  for (int i = NF; i < N; ++i) {
    V[i] = -V[i];
  }
}
template<> inline void Minus<1>(const int N, float *V) {
  if (N > 4) {
    V[1] = -V[1];
    V[2] = -V[2];
    V[3] = -V[3];
    Minus<0>(N - 4, V + 4);
  } else {
    for (int i = 1; i < N; ++i) {
      V[i] = -V[i];
    }
  }
}
template<> inline void Minus<2>(const int N, float *V) {
  if (N > 4) {
    V[2] = -V[2];
    V[3] = -V[3];
    Minus<0>(N - 4, V + 4);
  } else {
    for (int i = 2; i < N; ++i) {
      V[i] = -V[i];
    }
  }
}
template<> inline void Minus<3>(const int N, float *V) {
  if (N > 4) {
    V[3] = -V[3];
    Minus<0>(N - 4, V + 4);
  } else if (N == 4) {
    V[3] = -V[3];
  }
}
inline void Minus(const int i, const int N, float *V) {
  switch (i) {
  case 0: Minus<0>(N, V);     break;
  case 1: Minus<1>(N, V);     break;
  case 2: Minus<2>(N, V);     break;
  case 3: Minus<3>(N, V);     break;
  case 4: Minus<0>(N - 4, V + 4); break;
  case 5: Minus<1>(N - 4, V + 4); break;
  case 6: Minus<2>(N - 4, V + 4); break;
  default: {
    const int NF = SIMD_FLOAT_FLOOR(i);
    Minus(i - NF, N - NF, V + NF);
    break;
  }
  }
}
template<typename TYPE> inline void Minus(const int N, TYPE *V);
template<> inline void Minus<float>(const int N, float *V) { Minus<0>(N, V); }
template<> inline void Minus<double>(const int N, double *V) {
  for (int i = 0; i < N; ++i) {
    V[i] = -V[i];
  }
}

inline void Inverse(const int N, float *V, const float s = 1.0f, const bool chkZero = false,
                    const float eps = 0.0f) {
  xp128f _s = xp128f::get(s);
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f *v = (xp128f *) V;
  if (!chkZero) {
    for (int i = 0; i < N; ++i) {
      V[i] = s / V[i];
    }
  } else if (eps == 0.0f) {
    // TODO (yanghongtian) : NEON implemtation
    for (int i = 0; i < N; ++i) {
      V[i] = V[i] != 0.0f ? s / V[i] : 0.0f;
    }
  } else if (eps != 0.0f) {
    for (int i = 0; i < N; ++i) {
      if (V[i] <= eps) {
        V[i] = 0.0f;
      }
    }
    Inverse(N, V, s, true);
  }
}

template<typename TYPE> inline TYPE Sum(const int N, const TYPE *V);
template<> inline float Sum<float>(const int N, const float *V) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f S = xp128f::get(0.0f);
  const xp128f *v = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v) {
    S += *v;
  }
  for (int i = NF; i < N; ++i) {
    S[0] += V[i];
  }
  return S.vsum_all();
}

template<typename TYPE> inline TYPE Sum(const int i1, const int i2, const TYPE *V);
template<> inline float Sum<float>(const int i1, const int i2, const float *V) {
  if (i2 - i1 < 4) {
    float s = 0.0f;
    for (int i = i1; i < i2; ++i) {
      s += V[i];
    }
    return s;
  }
  const int i1c = SIMD_FLOAT_CEIL(i1);
  float S = Sum(i2 - i1c, V + i1c);
  for (int i = i1; i < i1c; ++i) {
    S += V[i];
  }
  return S;
}
template<typename TYPE> inline TYPE Mean(const int N, const TYPE *V) {
  return N == 0 ? 0 : Sum(N, V) / N;
}

template<typename TYPE> inline TYPE Variance(const int N, const TYPE *V, const TYPE u);
template<> inline float Variance(const int N, const float *V, const float u) {
  const xp128f _u = xp128f::get(u);
  xp128f d, Sd2 = xp128f::get(0.0f);
  const xp128f *v = (xp128f *) V;
  const int NF = SIMD_FLOAT_FLOOR(N);
  for (int i = 0; i < NF; i += 4, ++v) {
    d = *v - _u;
    Sd2 += d * d;
  }
  for (int i = NF; i < N; ++i) {
    d[0] = V[i] - u;
    Sd2[0] += d[0] * d[0];
  }
  return Sd2.vsum_all() / N;
}

template<typename TYPE> inline TYPE Variance(const int N, const TYPE *V) {
  const TYPE u = Mean(N, V);
  return Variance(N, V, u);
}

template<typename TYPE> inline TYPE Maximal(const int N, const TYPE *V);
template<typename TYPE> inline TYPE Minimal(const int N, const TYPE *V);
template<> inline float Maximal<float>(const int N, const float *V) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f m = xp128f::get(-FLT_MAX);
  const xp128f *v = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v) {
    m.vmax_with_other(*v);
  }
  float max_val = m.vmaximal();
  for (int i = NF; i < N; ++i) {
    max_val = std::max(V[i], max_val);
  }
  return max_val;
}
template<> inline float Minimal<float>(const int N, const float *V) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f m = xp128f::get(FLT_MAX);
  const xp128f *v = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v) {
    m.vmin_with_other(*v);
  }
  float min_val = m.vminimal();
  for (int i = NF; i < N; ++i) {
    min_val = std::min(V[i], min_val);
  }
  return min_val;
}

template<int SKIP> inline float Dot(const int N, const float *Va, const float *Vb);
template<> inline float Dot<0>(const int N, const float *Va, const float *Vb) {
  xp128f d = xp128f::get(0.0f);
  const xp128f *a = (xp128f *) Va, *b = (xp128f *) Vb;
  const int NF = SIMD_FLOAT_FLOOR(N);
  for (int i = 0; i < NF; i += 4, ++a, ++b) {
    d += (*a) * (*b);
  }
  for (int i = NF; i < N; ++i) {
    d[0] += Va[i] * Vb[i];
  }
  return d.vsum_all();
}
template<> inline float Dot<1>(const int N, const float *Va, const float *Vb) {
  if (N > 4) {
    const xp128f u = *((xp128f *) Va) * *((xp128f *)Vb);
    return u[1] + u[2] + u[3] + Dot<0>(N - 4, Va + 4, Vb + 4);
  } else {
    float d = 0.0f;
    for (int i = 1; i < N; ++i) {
      d = Va[i] * Vb[i] + d;
    }
    return d;
  }
}
template<> inline float Dot<2>(const int N, const float *Va, const float *Vb) {
  if (N > 4) {
    return Va[2] * Vb[2] + Va[3] * Vb[3] + Dot<0>(N - 4, Va + 4, Vb + 4);
  } else {
    float d = 0.0f;
    for (int i = 2; i < N; ++i) {
      d = Va[i] * Vb[i] + d;
    }
    return d;
  }
}
template<> inline float Dot<3>(const int N, const float *Va, const float *Vb) {
  if (N > 4) {
    return Va[3] * Vb[3] + Dot<0>(N - 4, Va + 4, Vb + 4);
  } else if (N == 4) {
    return Va[3] * Vb[3];
  } else {
    return 0.0f;
  }
}
inline float Dot(const int i, const int N, const float *Va, const float *Vb) {
  switch (i) {
  case 0: return Dot<0>(N, Va, Vb);
  case 1: return Dot<1>(N, Va, Vb);
  case 2: return Dot<2>(N, Va, Vb);
  case 3: return Dot<3>(N, Va, Vb);
  case 4: return Dot<0>(N - 4, Va + 4, Vb + 4);
  case 5: return Dot<1>(N - 4, Va + 4, Vb + 4);
  case 6: return Dot<2>(N - 4, Va + 4, Vb + 4);
  default: {
    const int NF = SIMD_FLOAT_FLOOR(i);
    return Dot(i - NF, N - NF, Va + NF, Vb + NF);
  }
  }
}
template<int SKIP> inline double Dot(const int N, const double *Va, const double *Vb) {
  double d = 0.0;
  for (int i = SKIP; i < N; ++i) {
    d += Va[i] * Vb[i];
  }
  return d;
}

template<typename TYPE> inline TYPE SquaredLength(const int N, const TYPE *V);
template<> inline float SquaredLength(const int N, const float *V) {
  xp128f s = xp128f::get(0.0f);
  const xp128f *v = (xp128f *) V;
  const int NF = SIMD_FLOAT_FLOOR(N);
  for (int i = 0; i < NF; i += 4, ++v) {
    s += (*v) * (*v);
  }
  float sum_val = s.vsum_all();
  for (int i = NF; i < N; ++i) {
    sum_val += V[i] * V[i];
  }
  return sum_val;
}

template<typename TYPE> inline void Square(const int N, TYPE *V);
template<> inline void Square(const int N, float *V) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f *v = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v) {
    *v *= (*v);
  }
  for (int i = NF; i < N; ++i) {
    V[i] *= V[i];
  }
}

template<typename TYPE> inline void SquareRoot(const int N, TYPE *V);
template<> inline void SquareRoot(const int N, float *V) {
  const int NF = SIMD_FLOAT_FLOOR(N);
  xp128f *v = (xp128f *) V;
  for (int i = 0; i < NF; i += 4, ++v) {
    v->vmake_sqrt();
  }
  for (int i = NF; i < N; ++i) {
    V[i] = sqrtf(V[i]);
  }
}
}  // namespace SIMD

#endif  // _UTILITY_SIMD_H_
