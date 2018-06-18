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
#ifndef _CANDIDATE_H_
#define _CANDIDATE_H_

template<typename TYPE>
class Candidate {
 public:
  inline Candidate() {}
  inline Candidate(const int idx, const TYPE score) { Set(idx, score); }
  inline void Set(const int idx, const TYPE score) { m_idx = idx; m_score = score; }
  inline void Get(int &idx, TYPE &score) const { idx = m_idx; score = m_score; }
  inline bool operator < (const Candidate &c) const { return m_score < c.m_score; }
  inline bool operator > (const Candidate &c) const { return m_score > c.m_score; }
 public:
  int m_idx;
  TYPE m_score;
};

template<class TYPE>
class CandidateVector {
 public:
  inline int Size() const { return int(m_data.size()); }
  inline void Resize(const int N) { m_data.resize(N); }
  inline void Push(const int idx, const TYPE score) { m_data.push_back(Candidate<TYPE>(idx, score)); }
  inline void MakeZero() {
    const int N = int(m_data.size());
    for (int i = 0; i < N; ++i)
      m_data[i].Set(i, 0);
  }
  inline void RemoveZero() {
    int i, j;
    const int N = int(m_data.size());
    for (i = j = 0; i < N; ++i) {
      if (m_data[i].m_score != 0)
        m_data[j++] = m_data[i];
    }
    m_data.resize(j);
  }
  inline void SortAscending() {
    std::sort(m_data.begin(), m_data.end());
  }
  inline void SortDescending() {
    std::sort(m_data.begin(), m_data.end(), std::greater<Candidate<TYPE> >());
  }
  inline void MarkFirstN(const int N, std::vector<ubyte> &marks, const ubyte mark) const {
    const int kN = std::min(Size(), N);
    for (int i = 0; i < kN; ++i)
      marks[m_data[i].m_idx] = mark;
  }
  inline void GetFirstN(const int N, std::vector<int> &idxs) const {
    const int kN = std::min(Size(), N);
    idxs.resize(kN);
    for (int i = 0; i < kN; ++i)
      idxs[i] = m_data[i].m_idx;
  }
 public:
  std::vector<Candidate<TYPE> > m_data;
};

#endif
