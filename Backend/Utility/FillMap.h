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
#ifndef _FILL_MAP_H_
#define _FILL_MAP_H_

#include "Table.h"

class FillMap : public Table<ubyte> {

public:

  inline void Initialize(const int w, const int h) { Resize(w, h); MakeZero(); }

  inline void Fill(const int x, const int y, const int dist = 0) {
    if (dist == 0) {
      if (x >= 0 && x < m_w && y >= 0 && y < m_h)
        m_rows[y][x] = UCHAR_MAX;
      return;
    }
    const int x1 = std::min(std::max(x - dist, 0), m_w), x2 = std::min(x + dist, m_w);
    const int y1 = std::min(std::max(y - dist, 0), m_h), y2 = std::min(y + dist, m_h);
    const int size = sizeof(ubyte) * (x2 - x1);
    for (int _y = y1; _y < y2; ++_y)
      memset(m_rows[_y] + x1, UINT_MAX, size);
  }
  inline bool Filled(const int x, const int y) const { return m_rows[y][x] != 0; }
  inline int CountFilled() const {
    int SN = 0;
    for (int y = 0; y < m_h; ++y)
      for (int x = 0; x < m_w; ++x) {
        if (m_rows[y][x])
          ++SN;
      }
    return SN;
  }

  inline void GetDilated(FillMap &FM, const int dist = 1) const {
    FM.Resize(m_w, m_h);
    for (int y = 0; y < m_h; ++y) {
      const int y1 = std::max(y - dist, 0), y2 = std::min(y + dist, m_h);
      for (int x = 0; x < m_w; ++x) {
        const int x1 = std::max(x - dist, 0), x2 = std::min(x + dist, m_w);
        ubyte v = 0;
        for (int _y = y1; _y < y2; ++_y)
          for (int _x = x1; _x < x2; ++_x)
            v = v | m_rows[_y][_x];
        FM.m_rows[y][x] = v;
      }
    }
  }
};

#endif
