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
#ifndef _CAMERA_TRAJECTORY_H_
#define _CAMERA_TRAJECTORY_H_

#include "Camera.h"
#include "AlignedVector.h"

#define CT_FLAG_DEFAULT         0
#define CT_FLAG_INVERSE         1
#define CT_FLAG_LEFT_HAND       2
#define CT_FLAG_XYZW            4
#define CT_FLAG_ORIGIN          8
#define CT_FLAG_ORIGIN_GRAVITY  16
#define CT_FLAG_TIME_RESET      32
#define CT_FLAG_MOTION          64
#define CT_FLAG_MOTION_BABW     128

class CameraTrajectory {

 public:

  inline CameraTrajectory(const ubyte flag = CT_FLAG_DEFAULT) : m_flag(flag) {}

  inline void operator = (const CameraTrajectory &CT) {
    m_flag = CT.m_flag;
    m_Cs.Set(CT.m_Cs);
    m_ts = CT.m_ts;
  }

  inline void Resize(const int N) { m_Cs.Resize(N); m_ts.resize(N); }
  inline int Size() const { return m_Cs.Size(); }
  inline bool Empty() const { return m_Cs.Empty(); }
  inline void Swap(CameraTrajectory &CT) {
    UT_SWAP(m_flag, CT.m_flag);
    m_Cs.Swap(CT.m_Cs);
    m_ts.swap(CT.m_ts);
  }
  inline void Push(const Camera &C, const float t) { m_Cs.Push(C); m_ts.push_back(t); }
  inline void Push(const CameraTrajectory &CT, const int i) { Push(CT.m_Cs[i], CT.m_ts[i]); }
  inline bool Load(const std::string fileName, const double tFactor = 1.0,
                   const double tFirst = 0.0, const std::vector<float> *ts = NULL,
                   const float dtMax = FLT_MAX, const ubyte flag = CT_FLAG_DEFAULT,
                   const Rigid3D *Ts = NULL, const Rotation3D *Ru = NULL) {
    m_flag = flag;
    Resize(0);
    FILE *fp = fopen(fileName.c_str(), "r");
    if (!fp) {
      return false;
    }

    double v;
    float t;
    Quaternion q;
    Camera C;
    std::vector<double> vs;
    C.m_v.Invalidate();
    C.m_ba.Invalidate();
    C.m_bw.Invalidate();
    m_flag &= ~CT_FLAG_MOTION;
    m_ba.MakeZero();
    m_bw.MakeZero();
    int SN = 0;
    char line[UT_STRING_WIDTH_MAX], *num;
    while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
      if (line[0] == '#') {
        continue;
      }
      vs.resize(0);
      num = strtok(line, " ,");
      while (num && sscanf(num, "%lf", &v) == 1) {
        vs.push_back(v);
        num = strtok(NULL, " ,");
      }
      const int n1 = static_cast<int>(vs.size());
      if (n1 == 4) {
        for (int i = 0; i < 8; ++i) {
          fscanf(fp, "%lf", &v);
          vs.push_back(v);
        }
        fscanf(fp, "\n");
        fgets(line, UT_STRING_WIDTH_MAX, fp);
      }
      const int n2 = static_cast<int>(vs.size());
#ifdef CFG_DEBUG
      UT_ASSERT(n2 == 1 || n2 == 7 || n2 == 8 || n2 == 12 || n2 == 13 || n2 == 17);
#endif
      const int i = Size();
      const double *_v = vs.data();
      if (n2 == 7 || n2 == 12) {
        if (ts) {
          t = ts->at(i);
        } else {
          t = float(i);
        }
      } else {
        if (ts && (flag & CT_FLAG_TIME_RESET)) {
          t = ts->at(i);
        } else {
          t = float(vs[0] * tFactor - tFirst);
        }
        ++_v;
      }
      if (n2 == 1) {
        fscanf(fp, "%f %f %f\n", &C.m_T.r00(), &C.m_T.r01(), &C.m_T.r02());
        fscanf(fp, "%f %f %f\n", &C.m_T.r10(), &C.m_T.r11(), &C.m_T.r12());
        fscanf(fp, "%f %f %f\n", &C.m_T.r20(), &C.m_T.r21(), &C.m_T.r22());
        fscanf(fp, "%f %f %f\n", &C.m_T.tx(), &C.m_T.ty(), &C.m_T.tz());
        if (flag & CT_FLAG_INVERSE) {
          C.m_T.Inverse();
        }
        C.m_T.GetPosition(C.m_p);
      } else if (n2 == 7 || n2 == 8 || n2 == 17) {
        C.m_p.Set(_v);
        if (flag & CT_FLAG_XYZW) {
          q.Set(_v + 3);
        } else {
          q.xyzw().vset_all_lane(float(_v[4]), float(_v[5]), float(_v[6]), float(_v[3]));
        }
        if (flag & CT_FLAG_INVERSE) {
          q.Inverse();
        }
        q.Normalize();
        C.m_T.Set(q, C.m_p);
        if (n2 == 17) {
          C.m_v.Set(_v + 7);
          if (flag & CT_FLAG_MOTION_BABW) {
            C.m_ba.Set(_v + 10);
            C.m_bw.Set(_v + 13);
          } else {
            C.m_bw.Set(_v + 10);
            C.m_ba.Set(_v + 13);
          }
          m_ba += C.m_ba;
          m_bw += C.m_bw;
          ++SN;
        }
      } else {
        C.m_T.Set(_v);
//#ifdef CFG_DEBUG
#if 0
        T.AssertOrthogonal();
#endif
        if (flag & CT_FLAG_INVERSE) {
          C.m_T.Inverse();
        }
        C.m_T.GetPosition(C.m_p);
      }
      if (flag & CT_FLAG_LEFT_HAND) {
        C.m_T.GetQuaternion(q);
        q.y() = -q.y();
        C.m_p.y() = -C.m_p.y();
        C.m_T.Set(q, C.m_p);
      }
//#ifdef CFG_DEBUG
#if 0
      UT_ASSERT(m_ts.empty() || t > m_ts.back());
#endif
      if (!m_ts.empty() && t < m_ts.back()) {
        continue;
      }
      m_Cs.Push(C);
      m_ts.push_back(t);
    }
    fclose(fp);
    UT::PrintLoaded(fileName);
    if (SN > 0) {
      m_flag |= CT_FLAG_MOTION;
      const xp128f s = xp128f::get(1.0f / SN);
      m_ba *= s;
      m_bw *= s;
    }
    const int N1 = Size();
    if (N1 == 0) {
      return false;
    }
    if (ts) {
      const int Nt = static_cast<int>(ts->size());
      CameraTrajectory CTGT(m_flag);
      //if (dtMax == FLT_MAX) {
      if (dtMax < 0.0f || dtMax == FLT_MAX) {
        CTGT.Resize(Nt);
        for (int i = 0; i < Nt; ++i) {
          const float t = ts->at(i);
          Interpolate(t, CTGT.m_Cs[i]); 
          CTGT.m_ts[i] = t;
        }
      } else {
        for (int i = 0; i < Nt; ++i) {
          const int j = Search(ts->at(i), dtMax);
          if (j != -1) {
            CTGT.Push(m_Cs[j], m_ts[j]);
          }
        }
      }
      Swap(CTGT);
    }
    const int N2 = Size();
    if (N2 == 0) {
      return false;
    }
    if (Ts) {
      TransformPose(*Ts, m_Cs);
    }
    if (Ru) {
      TransformBias(*Ru, m_Cs, m_ba, m_bw);
    }
    if (flag & CT_FLAG_ORIGIN) {
      AlignToOrigin(m_Cs, (flag & CT_FLAG_ORIGIN_GRAVITY) != 0, (m_flag & CT_FLAG_MOTION) != 0);
    }
    return true;
  }

  inline void MakeSubset(const std::vector<int> &idxs) {
    const int N = static_cast<int>(idxs.size());
#ifdef CFG_DEBUG
    for (int i = 1; i < N; ++i) {
      UT_ASSERT(idxs[i - 1] < idxs[i]);
    }
#endif
    for (int i = 0; i < N; ++i) {
      const int j = idxs[i];
      m_Cs[i] = m_Cs[j];
      m_ts[i] = m_ts[j];
    }
    Resize(N);
  }

  inline int Search(const float t, const float dtMax = 0.0f) const {
    const int N = static_cast<int>(m_ts.size());
    const int i2 = static_cast<int>(std::lower_bound(m_ts.begin(), m_ts.end(), t) -
                                                     m_ts.begin());
    const int i1 = i2 - 1;
    const float dt1 = i1 >= 0 && i1 < N ? fabs(m_ts[i1] - t) : FLT_MAX;
    const float dt2 = i2 >= 0 && i2 < N ? fabs(m_ts[i2] - t) : FLT_MAX;
    if (dt1 < dt2 && dt1 <= dtMax) {
      return i1;
    } else if (dt2 < dt1 && dt2 <= dtMax) {
      return i2;
    } else {
      return -1;
    }
  }

  inline int Search(const float t, Camera &Ci, float &ti, const float dtMax = 0.0f) const {
    const int i = Search(t, dtMax);
    if (i == -1) {
      Ci.Invalidate();
      ti = UT::Invalid<float>();
    } else {
      Ci = m_Cs[i];
      ti = m_ts[i];
    }
    return i;
  }

  inline void Interpolate(const float t, Camera &C) const {
#if 0
#ifdef CFG_DEBUG
    UT_ASSERT(m_ts.size() >= 2);
#endif
    int i1, i2;
    if (t <= m_ts.front()) {
      i1 = 0;
      i2 = 1;
    } else if (t >= m_ts.back()) {
      const int N = static_cast<int>(m_ts.size());
      i1 = N - 2;
      i2 = N - 1;
    } else {
      i2 = int(std::upper_bound(m_ts.begin(), m_ts.end(), t) - m_ts.begin());
      i1 = i2 - 1;
    }
#else
#ifdef CFG_DEBUG
    UT_ASSERT(!m_ts.empty());
#endif
    if (t <= m_ts.front()) {
      C = m_Cs.Front();
      return;
    } else if (t >= m_ts.back()) {
      C = m_Cs.Back();
      return;
    }
    const int i2 = int(std::upper_bound(m_ts.begin(), m_ts.end(), t) - m_ts.begin());
    const int i1 = i2 - 1;
#endif
    const float t1 = m_ts[i1], t2 = m_ts[i2];
    const float w1 = (t2 - t) / (t2 - t1);
    const Camera &C1 = m_Cs[i1], &C2 = m_Cs[i2];
    const Quaternion q1 = C1.m_T.GetQuaternion();
    const Quaternion q2 = C2.m_T.GetQuaternion();
    Quaternion q;
    q.Slerp(w1, q1, q2);
    C.m_p.Interpolate(w1, C1.m_p, C2.m_p);
    C.m_T.Set(q, C.m_p);
    if (m_flag & CT_FLAG_MOTION) {
      C.m_v.Interpolate(w1, C1.m_v, C2.m_v);
      C.m_ba.Interpolate(w1, C1.m_ba, C2.m_ba);
      C.m_bw.Interpolate(w1, C1.m_bw, C2.m_bw);
    } else {
      C.m_v.Invalidate();
      C.m_ba.Invalidate();
      C.m_bw.Invalidate();
    }
  }

  static inline void TransformPose(const Rigid3D &T, AlignedVector<Camera> &Cs) {
    const int N = Cs.Size();
    for (int i = 0; i < N; ++i) {
      Camera &C = Cs[i];
      C.m_T = T * C.m_T;
      C.m_T.GetPosition(C.m_p);
    }
  }
  static inline void TransformBias(const Rotation3D &R, AlignedVector<Camera> &Cs,
                                   LA::AlignedVector3f &ba, LA::AlignedVector3f &bw) {
    const int N = Cs.Size();
    for (int i = 0; i < N; ++i) {
      Camera &C = Cs[i];
      if (C.m_ba.Valid()) {
        C.m_ba = R.GetApplied(C.m_ba);
      }
      if (C.m_bw.Valid()) {
        C.m_bw = R.GetApplied(C.m_bw);
      }
    }
    ba = R.GetApplied(ba);
    bw = R.GetApplied(bw);
  }
  static inline void AlignToOrigin(AlignedVector<Camera> &Cs, const bool g = true,
                                   const bool m = true) {
    if (Cs.Empty()) {
      return;
    }
    Rigid3D Tr;
    if (g) {
      Rigid3D Tg;
      const Rigid3D &T0 = Cs[0].m_T;
      const LA::AlignedVector3f g = T0.GetGravity();
      Tg.MakeIdentity(&g);
      Tr = Tg.GetInverse() * T0;
    } else {
      Tr = Cs[0].m_T;
    }
    const int N = Cs.Size();
    for (int i = 0; i < N; ++i) {
      Camera &C = Cs[i];
      C.m_T = C.m_T / Tr;
      C.m_T.GetPosition(C.m_p);
      if (m) {
        C.m_v = Tr.GetAppliedRotation(C.m_v);
      }
    }
  }

 public:

  ubyte m_flag;
  LA::AlignedVector3f m_ba, m_bw;
  AlignedVector<Camera> m_Cs;
  std::vector<float> m_ts;

};

#endif
