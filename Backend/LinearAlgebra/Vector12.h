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
#ifndef _VECTOR_12_H_
#define _VECTOR_12_H_

#include "Vector6.h"
#include "Utility.h"

namespace LA {

class AlignedVector12f {

 public:
   
  inline const xp128f &v0123() const { return m_data4[0]; }      inline xp128f &v0123() { return m_data4[0]; }
  inline const xp128f &v4567() const { return m_data4[1]; }      inline xp128f &v4567() { return m_data4[1]; }
  inline const xp128f &v_8_9_10_11() const { return m_data4[2]; }
  inline       xp128f &v_8_9_10_11()       { return m_data4[2]; }
  inline const float& v0() const { return m_data[0]; }      inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }      inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }      inline float& v2() { return m_data[2]; }
  inline const float& v3() const { return m_data[3]; }      inline float& v3() { return m_data[3]; }
  inline const float& v4() const { return m_data[4]; }      inline float& v4() { return m_data[4]; }
  inline const float& v5() const { return m_data[5]; }      inline float& v5() { return m_data[5]; }
  inline const float& v6() const { return m_data[6]; }      inline float& v6() { return m_data[6]; }
  inline const float& v7() const { return m_data[7]; }      inline float& v7() { return m_data[7]; }
  inline const float& v8() const { return m_data[8]; }      inline float& v8() { return m_data[8]; }
  inline const float& v9() const { return m_data[9]; }      inline float& v9() { return m_data[9]; }
  inline const float& v10() const { return m_data[10]; }    inline float& v10() { return m_data[10]; }
  inline const float& v11() const { return m_data[11]; }    inline float& v11() { return m_data[11]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline void operator += (const AlignedVector12f &v) {
    m_data4[0] += v.m_data4[0];
    m_data4[1] += v.m_data4[1];
    m_data4[2] += v.m_data4[2];
  }

  inline void Set(const float *v) { memcpy(this, v, 48); }
  inline void Set(const AlignedVector3f &v0, const AlignedVector3f &v1,
                  const AlignedVector3f &v2, const AlignedVector3f &v3) {
    memcpy(&this->v0(), v0, 12);
    memcpy(&this->v3(), v1, 12);
    memcpy(&this->v6(), v2, 12);
    memcpy(&this->v9(), v3, 12);
  }
  inline void Set(const Vector6f &v0, const Vector6f &v1) {
    memcpy(&this->v0(), v0, 24);
    memcpy(&this->v6(), v1, 24);
  }
  inline void Get(float *v) const { memcpy(v, this, 48); }
  inline void Get(ProductVector6f &v0, ProductVector6f &v1) const {
    v0.Set(&this->v0());
    v1.Set(&this->v6());
  }
  inline void GetMinus(AlignedVector12f &v) const {
    const xp128f zero = xp128f::get(0.0f);
    v.m_data4[0] = zero - m_data4[0];
    v.m_data4[1] = zero - m_data4[1];
    v.m_data4[2] = zero - m_data4[2];
    v.m_data4[3] = zero - m_data4[3];
  }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedVector12f)); }

  inline float SquaredLength() const {
    return (m_data4[0] * m_data4[0] + m_data4[1] * m_data4[1] +
            m_data4[2] * m_data4[2]).vsum_all();
  }

 protected:
  union {
    xp128f m_data4[3];
    float m_data[12];
  };
};

}  // namespace LA
#endif
