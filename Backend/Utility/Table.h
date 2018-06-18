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
#ifndef _TABLE_H_
#define _TABLE_H_

#include "Utility.h"

template<class TYPE>
class Table {

public:
  
  inline Table() : m_w(0), m_h(0) {}

  inline const TYPE* operator[] (const int i) const { return m_rows[i]; }
  inline       TYPE* operator[] (const int i)       { return m_rows[i]; }

  inline const TYPE* Data() const { return m_data.empty() ? NULL : m_data.data(); }
  inline       TYPE* Data()       { return m_data.empty() ? NULL : m_data.data(); }

  inline const int& w() const { return m_w; }
  inline const int& h() const { return m_h; }
  inline bool Empty() const { return m_data.empty(); }
  inline int Size() const { return int(m_data.size()); }
  inline void Resize(const int w, const int h) {
    if (m_w == w && m_h == h)
      return;
    m_w = w;
    m_h = h;
    const int N = m_w * m_h;
    m_data.resize(N);
    m_rows.resize(m_h);
    m_rows[0] = m_data.data();
    for (int y = 1; y < m_h; ++y)
      m_rows[y] = m_rows[y - 1] + m_w;
  }

  inline void operator = (const Table<TYPE> &T) {
    Resize(T.m_w, T.m_h);
    memcpy(m_data.data(), T.m_data.data(), sizeof(TYPE) * m_data.size());
  }

  inline void MakeZero() { memset(m_data.data(), 0, sizeof(TYPE) * m_data.size()); }

  inline void Swap(Table<TYPE> &T) {
    UT_SWAP(m_w, T.m_w);
    UT_SWAP(m_h, T.m_h);
    m_data.swap(T.m_data);
    m_rows.swap(T.m_rows);
  }

  inline void SaveB(FILE *fp) const {
    UT::SaveB(m_w, fp);
    UT::SaveB(m_h, fp);
    UT::VectorSaveB<TYPE>(m_data, fp);
  }
  inline void LoadB(FILE *fp) {
    const int w = UT::LoadB<int>(fp);
    const int h = UT::LoadB<int>(fp);
    Resize(w, h);
    UT::VectorLoadB<TYPE>(m_data, fp);
  }
  inline float MemoryMB() const {
    return UT::MemoryMB<int>(2)
         + UT::VectorMemoryMB<TYPE>(m_data)
         + UT::VectorMemoryMB<TYPE *>(m_rows);
  }

protected:

  int m_w, m_h;
  std::vector<TYPE> m_data;
  std::vector<TYPE *> m_rows;
};

#endif
