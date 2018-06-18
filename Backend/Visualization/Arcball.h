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
#ifndef _ARCBALL_H_
#define _ARCBALL_H_

#include "Rotation.h"
#include "Matrix4x4.h"
#include <GL/glew.h>
#include <GL/glut.h>

class Arcball {

 public:

  inline void Initialize() {
    m_C.MakeZero();
    m_OC = m_OC2 = 0.0f;
    m_nC.MakeZero();
    //m_r = m_r2 = 0.0f;
    SetRadius(0.0f);
    m_R.MakeIdentity();
    m_R.GetTranspose(m_RT);
    m_M = m_R;
    m_Cstart = m_C;
    m_Rstart = m_R;
  }
  inline void Initialize(const Rotation3D &R, const float zCenter, const float radius) {
    m_C.Set(0, 0, zCenter);
    m_OC = fabs(zCenter);
    m_OC2 = zCenter * zCenter;
    m_nC.Set(0.0f, 0.0f, 1.0f);
    SetRadius(radius);
    m_R = R;
    m_R.GetTranspose(m_RT);
    m_M = m_R;
    m_Cstart = m_C;
    m_rStart = m_r;
    m_Rstart = m_R;
  }
  inline void Initialize(const float zCenter, const float radius) {
    m_C.Set(0, 0, zCenter);
    m_OC = fabs(zCenter);
    m_OC2 = zCenter * zCenter;
    m_nC.Set(0.0f, 0.0f, 1.0f);
    SetRadius(radius);
    m_R.MakeIdentity();
    m_R.GetTranspose(m_RT);
    m_M = m_R;
    m_Cstart = m_C;
    m_rStart = m_r;
    m_Rstart = m_R;
  }
  inline void Initialize(const Point3D &center, const float radius) {
    m_C = center;
    m_OC2 = m_C.SquaredLength();
    m_OC = sqrtf(m_OC2);
    m_nC = m_C / m_OC;
    SetRadius(radius);
    m_R.MakeIdentity();
    m_R.GetTranspose(m_RT);
    m_M = m_R;
    m_Cstart = m_C;
    m_rStart = m_r;
    m_Rstart = m_R;
  }
  inline void Set(const float centerZ, const float radius) {
    m_C.Set(0, 0, centerZ);
    m_OC = centerZ;
    m_OC2 = m_OC * m_OC;
    m_nC.Set(0, 0, 1);
    SetRadius(radius);
  }
  inline const Point3D& GetCenter() const { return m_C; }
  inline void SetCenter(const Point3D &C) {
    m_C = C;
    m_OC2 = m_C.SquaredLength();
    m_OC = sqrtf(m_OC2);
    m_nC = m_C / m_OC;
    ComputeTangentFoot();
  }
  inline float GetCenterZ() const { return m_C.z(); }
  inline float GetRadius() const { return m_r; }
  inline const Rotation3D& GetRotation() const { return m_RT; }
  inline const Rotation3D& GetRotationTranspose() const { return m_R; }

  inline void StartChangingRadius() { m_rStart = m_r; }
  inline void ChangeRadius(const float ratio) { SetRadius(m_rStart * ratio); }
  inline void StartTranslatingCenter() { m_Cstart = m_C; }
  inline void TranslateCenter(const Point3D &dC) {
    m_C = dC + m_Cstart;
    m_OC2 = m_C.SquaredLength();
    m_OC = sqrtf(m_OC2);
    m_nC = m_C / m_OC;
    ComputeTangentFoot();
  }
  inline void TranslateCenterXY(const float dx, const float dy) {
    m_C.x() = dx + m_Cstart.x();
    m_C.y() = dy + m_Cstart.y();
    m_OC2 = m_C.SquaredLength();
    m_OC = sqrtf(m_OC2);
    m_nC = m_C / m_OC;
    ComputeTangentFoot();
  }
  inline void StartRotation(const Point3D &Xwin) {
    m_Rstart = m_R;
    ComputeCenterToIntersectionDirection(Xwin, m_nCXstart);
  }
  inline void ComputeRotation(const Point3D &Xwin) {
    Point3D nCX;
    ComputeCenterToIntersectionDirection(Xwin, nCX);

    Rotation3D dR;
    AxisAngle kth;
    kth.FromVectors(m_nCXstart, nCX);
    dR.SetAxisAngle(kth);

    m_R = m_Rstart * dR;
    m_R.GetTranspose(m_RT);
    m_M = m_R;
  }
  inline void ApplyTransformation() const {
    glTranslatef(m_C.x(), m_C.y(), m_C.z());
    glMultMatrixf(m_M);
    glTranslatef(-m_C.x(), -m_C.y(), -m_C.z());
  }
  inline void Draw(const int slices, const int stacks) const {
    glPushMatrix();
    glTranslatef(m_C.x(), m_C.y(), m_C.z());
    glMultMatrixf(m_M);
    glutWireSphere(double(m_r), slices, stacks);
    glPopMatrix();
  }

 protected:

  inline void SetRadius(const float radius) {
    m_r = radius;
    m_r2 = radius * radius;
    ComputeTangentFoot();
  }
  inline void ComputeTangentFoot() {
    m_OTfoot = m_OC - m_r2 / m_OC;
    m_Tfoot = m_nC * m_OC;
  }
  inline void ComputeCenterToIntersectionDirection(const Point3D &Xwin,
                                                   LA::AlignedVector3f &nCX) const {
    float a = Xwin.SquaredLength();
    float b = -Xwin.Dot(m_C);
    float c = m_OC2 - m_r2;
    float root = b * b - a * c;
    float t;
    if (root > 0) {
      t = (-b - sqrtf(root)) / a;
      nCX = Xwin * t - m_C;
      nCX.Normalize();
    } else {
      // Tfoot is perpendicular foot of the tangent point T on OC
      // Xext is the intersection of OXwin and TfootT
      // |OXext| / |OTfoot| = |OXwin| / |OXfoot|
      const Point3D Xext = Xwin * (m_OTfoot / Xwin.Dot(m_nC));

      // T = Xext + XextTfoot * t
      // TC = XextT - CXext = XextTfoot * t - CXext
      // |TC| = |XextTfoot * t - CXext| = R
      //Point3D CXext, XextTfoot;
      const Point3D CXext = Xext - m_C, XextTfoot = m_Tfoot - Xext;
      a = b = XextTfoot.SquaredLength();
      c = CXext.SquaredLength() - m_r2;
      t = (-b - sqrtf(b * b - a * c)) / a;
      nCX = XextTfoot * t + Xext - m_C;
      nCX.Normalize();
    }
  }

 private:

  Point3D m_C, m_Cstart, m_Tfoot;
  LA::AlignedVector3f m_nC, m_nCXstart;
  Rotation3D m_R, m_RT, m_Rstart;
  LA::AlignedMatrix4x4f m_M;
  float m_OC, m_OC2, m_r, m_r2, m_rStart, m_OTfoot;

};

#endif
