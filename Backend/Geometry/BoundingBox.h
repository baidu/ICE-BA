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
#ifndef _BOUNDING_BOX_H_
#define _BOUNDING_BOX_H_

#include "Point.h"

class BoundingBox2D {

 public:

  inline BoundingBox2D() {}
  inline BoundingBox2D(const float xMin, const float yMin, const float xMax,
                       const float yMax) : m_xMin(xMin, yMin), m_xMax(xMax, yMax) {}
  inline BoundingBox2D(const Point2D &xMin, const Point2D &xMax) : m_xMin(xMin), m_xMax(xMax) {}
  inline void Initialize() { m_xMin.Set(FLT_MAX, FLT_MAX); m_xMax.Set(-FLT_MAX, -FLT_MAX); }
  inline void Include(const Point2D &x) {
    if (x.x() < m_xMin.x()) {
      m_xMin.x() = x.x();
    }
    if (x.y() < m_xMin.y()) {
      m_xMin.y() = x.y();
    }
    if (x.x() > m_xMax.x()) {
      m_xMax.x() = x.x();
    }
    if (x.y() > m_xMax.y()) {
      m_xMax.y() = x.y();
    }
  }
  inline void Set(const Point2D &xMin, const Point2D &xMax) { m_xMin = xMin; m_xMax = xMax; }
  inline void Get(Point2D &xMin, Point2D &xMax) const { xMin = m_xMin; xMax = m_xMax; }
  inline bool Inside(const Point2D &x) const { return x.x() >= m_xMin.x() && x.y() >= m_xMin.y() && x.x() <= m_xMax.x() && x.y() <= m_xMax.y(); }
  inline bool Outside(const Point2D &x) const { return x.x() < m_xMin.x() || x.y() < m_xMin.y() || x.x() > m_xMax.x() || x.y() > m_xMax.y(); }
  //inline float ComputeDiagonalLength() const { return sqrtf(m_xMin.SquaredDistance(m_xMax)); }
  //inline float ComputeDiagonalSquaredLength() const { return m_xMin.SquaredDistance(m_xMax); }

 public:

  Point2D m_xMin, m_xMax;

};

#endif
