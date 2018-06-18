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
#ifndef _FRUSTRUM_H_
#define _FRUSTRUM_H_

#include "Rigid.h"
#include <GL/glew.h>
#include <GL/glut.h>

class Frustrum {

 public:

  inline void SetShape(const float z, const float xzRatio, const float yzRatio, const float size,
                       const float axisLen) {
    m_Xc.Set(z * xzRatio, z * yzRatio, z);
    SetSize(size);
    SetAxisLength(axisLen);
  }
  inline void SetSize(const float size) { m_XcScaled = m_Xc * size; }
  inline void SetAxisLength(const float len) { m_len.vdup_all_lane(m_Xc.z() * len); }
  inline void SetPose(const Rigid3D &T, const bool axis) {
    T.GetPosition(m_Xws[0]);

    // Xw = R^T * (Xc - t) = R^T * Xc + Cc
    xp128f XcR0 = T.r_00_01_02_x() * m_XcScaled.x();
    xp128f YcR1 = T.r_10_11_12_x() * m_XcScaled.y();
    xp128f ZcR2 = T.r_20_21_22_x() * m_XcScaled.z();

    // 4 plane vertexes' world coordinate are: X[R0] + Y[R1] + Z[R2] + Cc,   X[R0] - Y[R1] + Z[R2] + Cc,
    //                                       - X[R0] + Y[R1] + Z[R2] + Cc, - X[R0] - Y[R1] + Z[R2] + Cc,
    ZcR2 += m_Xws[0].xyzw();   //   Z[R2] + Cc
    m_Xws[1].xyzw() = XcR0;
    XcR0 += YcR1;              //   X[R0] + Y[R1]
    YcR1 = m_Xws[1].xyzw() - YcR1;   //   X[R0] - Y[R1]
    m_Xws[1].xyzw() = ZcR2 + XcR0;   //   X[R0] + Y[R1] + Z[R2] + Cc
    m_Xws[2].xyzw() = ZcR2 + YcR1;   //   X[R0] - Y[R1] + Z[R2] + Cc
    m_Xws[3].xyzw() = ZcR2 - YcR1;   // - X[R0] + Y[R1] + Z[R2] + Cc
    m_Xws[4].xyzw() = ZcR2 - XcR0;   // - X[R0] - Y[R1] + Z[R2] + Cc

    if (axis) {
      m_axisX.xyzw() = T.r_00_01_02_x() * m_len + m_Xws[0].xyzw();
      m_axisY.xyzw() = T.r_10_11_12_x() * m_len + m_Xws[0].xyzw();
      m_axisZ.xyzw() = T.r_20_21_22_x() * m_len + m_Xws[0].xyzw();
    }
  }
  inline const Point3D& GetPosition() const { return m_Xws[0]; }
  inline void MakeOrigin(const bool axis) {
    m_Xws[0].MakeZero();
    m_Xws[1].Set( m_XcScaled.x(),  m_XcScaled.y(), m_XcScaled.z());
    m_Xws[2].Set( m_XcScaled.x(), -m_XcScaled.y(), m_XcScaled.z());
    m_Xws[3].Set(-m_XcScaled.x(),  m_XcScaled.y(), m_XcScaled.z());
    m_Xws[4].Set(-m_XcScaled.x(), -m_XcScaled.y(), m_XcScaled.z());
    if (axis) {
      m_axisX.Set(m_len[0], 0.0f, 0.0f);
      m_axisY.Set(0.0f, m_len[0], 0.0f);
      m_axisZ.Set(0.0f, 0.0f, m_len[0]);

    }
  }
  inline void Draw(const bool axis) const {
    // [0;0;0], [X;Y;Z], [X;-Y;Z], [-X;Y;Z], [-X;-Y;Z]
    glBegin(GL_LINE_LOOP);
    glVertex3fv(m_Xws[0]);
    glVertex3fv(m_Xws[1]);
    glVertex3fv(m_Xws[3]);
    glVertex3fv(m_Xws[4]);
    glVertex3fv(m_Xws[2]);
    glEnd();
    glBegin(GL_LINES);
    glVertex3fv(m_Xws[0]);
    glVertex3fv(m_Xws[3]);
    glEnd();
    glBegin(GL_LINES);
    glVertex3fv(m_Xws[0]);
    glVertex3fv(m_Xws[4]);
    glEnd();
    glBegin(GL_LINES);
    glVertex3fv(m_Xws[1]);
    glVertex3fv(m_Xws[2]);
    glEnd();
    if (axis)
      DrawAxis();
  }
  inline void DrawAxis() const {
    glColor3ub(255, 0, 0);  glBegin(GL_LINES);  glVertex3fv(m_Xws[0]);  glVertex3fv(m_axisX); glEnd();
    glColor3ub(0, 255, 0);  glBegin(GL_LINES);  glVertex3fv(m_Xws[0]);  glVertex3fv(m_axisY); glEnd();
    glColor3ub(0, 0, 255);  glBegin(GL_LINES);  glVertex3fv(m_Xws[0]);  glVertex3fv(m_axisZ); glEnd();
  }
  template<int CHANNELS>
  inline void DrawTexture(const TextureGL<CHANNELS> &tex) const {
    tex.Bind();
    glBegin(GL_QUADS);
    glTexCoord2i(0, 0);                             glVertex3fv(m_Xws[4]);
    glTexCoord2i(0, tex.GetHeight());               glVertex3fv(m_Xws[3]);
    glTexCoord2i(tex.GetWidth(), tex.GetHeight());  glVertex3fv(m_Xws[1]);
    glTexCoord2i(tex.GetWidth(), 0);                glVertex3fv(m_Xws[2]);
    glEnd();
  }

 protected:

  Point3D m_Xc, m_XcScaled, m_Xws[5], m_axisX, m_axisY, m_axisZ;
  xp128f m_len;

};

#endif
