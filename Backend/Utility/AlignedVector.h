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
#ifndef _ALIGNED_VECTOR_H_
#define _ALIGNED_VECTOR_H_

#include "Utility.h"

template <class TYPE, const int GROWTH = 0, const int GROWTH_MAX = 1024>
class AlignedVector {

 public:

  inline AlignedVector() {
    m_own = true;
    m_data = NULL;
    m_N = m_capacity = 0;
  }
  inline AlignedVector(const int N) {
    m_own = true;
    m_data = SIMD::Malloc<TYPE>(N);
    m_N = m_capacity = N;
  }

  inline const TYPE& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }

  inline TYPE& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  //inline AlignedVector(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
  //{
  //  m_own = true;
  //  m_data = NULL;
  //  m_N = m_capacity = 0;
  //  Set(V);
  //}
  inline AlignedVector(AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) {
    m_own = true;
    m_data = NULL;
    m_N = m_capacity = 0;
    Bind(V.Data(), V.Size());
  }
  inline AlignedVector(void *V, const int N, const bool own = true) {
    m_own = true;
    m_data = NULL;
    m_N = m_capacity = 0;
    if (own) {
      Set((TYPE *) V, N);
    } else {
      Bind(V, N);
    }
  }
  inline ~AlignedVector() {
    if (m_data && m_own) {
      SIMD::Free<TYPE>(m_data);
    }
  }
  inline void operator = (const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) {
    //Bind(V.Data(), V.Size());
    m_own = false;
    m_data = V.m_data;
    m_N = V.m_N;
    m_capacity = V.m_capacity;
  }
  inline void Resize(const int N, const bool retain = false) {
    if (N <= m_capacity) {
      m_N = N;
    } else {
      TYPE *dataBkp = m_data;
      m_data = SIMD::Malloc<TYPE>(N);
      if (dataBkp) {
        if (retain) {
          memcpy(m_data, dataBkp, sizeof(TYPE) * m_N);
        }
        if (m_own) {
          SIMD::Free<TYPE>(dataBkp);
        }
      }
      m_own = true;
      m_N = m_capacity = N;
    }
  }
  inline void Reserve(const int N) {
    Clear();
    m_data = SIMD::Malloc<TYPE>(N);
    m_capacity = N;
  }
  inline void Clear() {
    if (m_data && m_own) {
      SIMD::Free<TYPE>(m_data);
    }
    m_own = true;
    m_data = NULL;
    m_N = m_capacity = 0;
  }
  inline bool Empty() const { return m_N == 0; }
  inline TYPE& Push() {
    if (m_N == m_capacity) {
      const int growth = GROWTH == 0 ? std::max(std::min(m_capacity, GROWTH_MAX), 1) : GROWTH;
      m_capacity += growth;
      TYPE *dataBkp = m_data;
      m_data = SIMD::Malloc<TYPE>(m_capacity);
      if (dataBkp) {
        memcpy(m_data, dataBkp, sizeof(TYPE) * m_N);
        if (m_own) {
          SIMD::Free<TYPE>(dataBkp);
        }
      }
      m_own = true;
    }
    return m_data[m_N++];
  }
  inline void Push(const TYPE &v) { Push() = v; }
  inline void Push(const TYPE *V, const int N) {
    if (m_N + N > m_capacity) {
      const int growth = N + GROWTH - (GROWTH % N);
      m_capacity += growth;
      TYPE *dataBkp = m_data;
      m_data = SIMD::Malloc<TYPE>(m_capacity);
      if (dataBkp) {
        memcpy(m_data, dataBkp, sizeof(TYPE) * m_N);
        if (m_own) {
          SIMD::Free<TYPE>(dataBkp);
        }
      }
      m_own = true;
    }
    memcpy(m_data + m_N, V, sizeof(TYPE) * N);
    m_N += N;
  }
  inline void Push(const AlignedVector<TYPE> &V) { Push(V.Data(), V.Size()); }
  inline void Pop(const int N, AlignedVector<float> *work) {
    const int N1 = Size(), N2 = N1 - N;
    work->Set((float *) (Data() + N), N2 * sizeof(TYPE) / sizeof(float));
    Set((TYPE *) work->Data(), N2);
  }
  inline void Insert(const int i) {
    const int N = Size();
    Resize(N + 1, true);
    for (int i1 = N - 1, i2 = N; i1 >= i; i2 = i1--) {
      m_data[i2] = m_data[i1];
    }
  }
  inline void Insert(const int i, const int N, AlignedVector<float> *work) {
    const int _N = Size() - i;
    if (_N > 0) {
      work->Set((float *) (Data() + i), _N * sizeof(TYPE) / sizeof(float));
      Resize(i + N, true);
      Push((TYPE *) work->Data(), _N);
    } else {
      Resize(i + N, true);
    }
  }
  inline void Insert(const int i, const AlignedVector<TYPE> &V, AlignedVector<float> *work) {
    const int _N = Size() - i;
    if (_N > 0) {
      work->Set((float *) (Data() + i), _N * sizeof(TYPE) / sizeof(float));
      Resize(i);
      Push(V);
      Push((TYPE *) work->Data(), _N);
    } else {
      Push(V);
    }
  }
  inline void Insert(const int i, const TYPE &v) {
    Insert(i);
    m_data[i] = v;
  }
  inline void InsertZero(const int i) {
    Insert(i);
    memset(m_data + i, 0, sizeof(TYPE));
  }
  inline void InsertZero(const int i, const int N, AlignedVector<float> *work) {
    Insert(i, N, work);
    memset(m_data + i, 0, sizeof(TYPE) * N);
  }
  inline void Erase(const int i, const int N = 1) {
    const int _N = Size() - N;
    for (int j = i; j < _N; ++j) {
      m_data[j] = m_data[j + N];
    }
    Resize(_N);
  }
  inline void Erase(const int N, AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) {
#ifdef CFG_DEBUG
    UT_ASSERT(N <= Size());
#endif
    V.Set(Data() + N, Size() - N);
    Resize(N);
    Swap(V);
  }
  inline void MakeZero() { memset(Data(), 0, sizeof(TYPE) * Size()); }
  inline void MakeZero(const int i1, const int N) {
    memset(Data() + i1, 0, sizeof(TYPE) * N);
  }

  inline void Set(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) {
    const int N = V.Size();
    Resize(N);
    memcpy(m_data, V.Data(), sizeof(TYPE) * N);
  }
  inline void Set(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V, const std::vector<int> &iVs) {
    const int N = int(iVs.size());
    Resize(N);
    for (int i = 0; i < N; ++i)
      m_data[i] = V[iVs[i]];
  }
  inline void Set(const TYPE *V) { memcpy(m_data, V, sizeof(TYPE) * m_N); }
  inline void Set(const TYPE *V, const int N) {
    Resize(N);
    memcpy(Data(), V, sizeof(TYPE) * N);
  }
  inline AlignedVector<TYPE, GROWTH, GROWTH_MAX> GetBlock(const int N) {
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 0 && N <= m_N);
#endif
    AlignedVector<TYPE, GROWTH, GROWTH_MAX> V;
    V.m_own = false;
    V.m_data = m_data;
    V.m_N = V.m_capacity = N;
    return V;
  }
  inline AlignedVector<TYPE, GROWTH, GROWTH_MAX> GetBlock(const int i, const int N) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && N >= 0 && i + N <= m_N);
#endif
    AlignedVector<TYPE, GROWTH, GROWTH_MAX> V;
    V.m_own = false;
    V.m_data = m_data + i;
    V.m_N = V.m_capacity = N;
    return V;
  }

  inline void Copy(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) {
#ifdef CFG_DEBUG
    UT_ASSERT(Size() == V.Size());
#endif
    memcpy(m_data, V.Data(), sizeof(TYPE) * Size());
  }
  inline void Bind(void *V, const int N) {
    if (m_data && m_own) {
      SIMD::Free<TYPE>(m_data);
    }
    m_data = (TYPE *) V;
    m_N = m_capacity = N;
    m_own = false;
  }
  inline void* BindNext() { return m_data + m_capacity; }
  inline int BindSize(const int N) const { return sizeof(TYPE) * N; }
  inline void Get(TYPE *V) const { memcpy(V, m_data, sizeof(TYPE) * m_N); }
  inline void Swap(AlignedVector<TYPE, GROWTH, GROWTH_MAX>& V) {
    UT_SWAP(m_own, V.m_own);
    UT_SWAP(m_data, V.m_data);
    UT_SWAP(m_N, V.m_N);
    UT_SWAP(m_capacity, V.m_capacity);
  }
  inline void Concatenate(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V1,
                          const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V2) {
    Resize(V1.Size() + V2.Size());
    memcpy(m_data, V1.Data(), sizeof(TYPE) * V1.Size());
    memcpy(m_data + V1.Size(), V2.Data(), sizeof(TYPE) * V2.Size());
  }

  inline bool operator == (const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) const {
    return Size() == V.Size() && UT::VectorEqual(Data(), V.Data(), Size());
  }
  inline const TYPE& operator[] (const int i) const {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i < m_N);
#endif
    return m_data[i];
  }
  inline TYPE& operator[] (const int i) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i < m_N);
#endif
    return m_data[i];
  }
  inline const TYPE* Data() const { return m_data; }
  inline       TYPE* Data()       { return m_data; }
  inline const TYPE* End() const { return m_data + m_N; }
  inline       TYPE* End()       { return m_data + m_N; }
  inline const TYPE& Front() const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_N != 0);
#endif
    return m_data[0];
  }
  inline TYPE& Front() {
#ifdef CFG_DEBUG
    UT_ASSERT(m_N != 0);
#endif
    return m_data[0];
  }
  inline const TYPE& Back() const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_N != 0);
#endif
    return m_data[m_N - 1];
  }
  inline TYPE& Back() {
#ifdef CFG_DEBUG
    UT_ASSERT(m_N != 0);
#endif
    return m_data[m_N - 1];
  }

  inline int Size() const { return m_N; }
  inline int Capacity() const { return m_capacity; }
  inline void SaveB(FILE *fp) const {
    UT::SaveB<int>(m_N, fp);
    UT::SaveB<TYPE>(m_data, m_N, fp);
  }
  inline void LoadB(FILE *fp) {
    const int N = UT::LoadB<int>(fp);
    Resize(N);
    UT::LoadB<TYPE>(m_data, N, fp);
  }
  inline bool SaveB(const std::string fileName) const {
    FILE *fp = fopen(fileName.c_str(), "wb");
    if (!fp) {
      return false;
    }
    SaveB(fp);
    fclose(fp);
    UT::PrintSaved(fileName);
    return true;
  }
  inline bool LoadB(const std::string fileName) {
    FILE *fp = fopen(fileName.c_str(), "rb");
    if (!fp) {
      return false;
    }
    LoadB(fp);
    fclose(fp);
    UT::PrintLoaded(fileName);
    return true;
  }

  inline float MemoryMB() const { return m_own ? 0 : UT::MemoryMB<TYPE>(m_capacity); }

 protected:

  bool m_own;
  TYPE* m_data;
  int m_N, m_capacity;

};

#endif
