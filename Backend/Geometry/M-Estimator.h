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
#ifndef _M_ESTIMATOR_H_
#define _M_ESTIMATOR_H_

#include "Utility.h"
#include "VectorN.h"

#define ME_VARIANCE_HUBER             1.809025f     // 1.345^2
#define ME_VARIANCE_TUKEY             21.95016201f  // 4.6851^2
#define ME_VARIANCE_TUKEY_INVERSE     0.0455577503f
#define ME_VARIANCE_STUDENT_T         5.68774801f   // 2.3849^2
#define ME_VARIANCE_STUDENT_T_INVERSE 0.1758165091f

namespace ME {

enum Function { FUNCTION_NONE, FUNCTION_HUBER, FUNCTION_TUKEY, FUNCTION_STUDENT_T_5, FUNCTION_CAUCHY };

template<int FUNCTION> inline float Weight(const float r2);
template<int FUNCTION> inline void Weight(const AlignedVector<float> &r2s,
                                          AlignedVector<float> &ws);
template<int FUNCTION> inline float Cost(const float r2);
template<int FUNCTION> inline float Cost(const AlignedVector<float> &r2s);
template<int FUNCTION> inline float Variance() { return FLT_MAX; }

//inline float Weight(const int func, const float r2)
//{
//  switch(func)
//  {
//  case FUNCTION_NONE:     return Weight<FUNCTION_NONE>(r2);
//  case FUNCTION_HUBER:    return Weight<FUNCTION_HUBER>(r2);
//  case FUNCTION_TUKEY:    return Weight<FUNCTION_TUKEY>(r2);
//  case FUNCTION_STUDENT_T_5:  return Weight<FUNCTION_STUDENT_T_5>(r2);
//  case FUNCTION_CAUCHY:   return Weight<FUNCTION_CAUCHY>(r2);
//  }
//  return 0.0f;
//}
//inline void Weight(const int func, const AlignedVector<float> &r2s, AlignedVector<float> &ws)
//{
//  switch(func)
//  {
//  case FUNCTION_NONE:     Weight<FUNCTION_NONE>(r2s, ws);     break;
//  case FUNCTION_HUBER:    Weight<FUNCTION_HUBER>(r2s, ws);    break;
//  case FUNCTION_TUKEY:    Weight<FUNCTION_TUKEY>(r2s, ws);    break;
//  case FUNCTION_STUDENT_T_5:  Weight<FUNCTION_STUDENT_T_5>(r2s, ws);  break;
//  case FUNCTION_CAUCHY:   Weight<FUNCTION_CAUCHY>(r2s, ws);   break;
//  }
//}
//inline float Cost(const int func, const AlignedVector<float> &r2s)
//{
//  switch(func)
//  {
//  case FUNCTION_NONE:     return Cost<FUNCTION_NONE>(r2s);
//  case FUNCTION_HUBER:    return Cost<FUNCTION_HUBER>(r2s);
//  case FUNCTION_TUKEY:    return Cost<FUNCTION_TUKEY>(r2s);
//  case FUNCTION_STUDENT_T_5:  return Cost<FUNCTION_STUDENT_T_5>(r2s);
//  case FUNCTION_CAUCHY:   return Cost<FUNCTION_CAUCHY>(r2s);
//  }
//  return 0.0f;
//}
//inline float Cost(const int func, const float r2)
//{
//  switch(func)
//  {
//  case FUNCTION_NONE:     return Cost<FUNCTION_NONE>(r2);
//  case FUNCTION_HUBER:    return Cost<FUNCTION_HUBER>(r2);
//  case FUNCTION_TUKEY:    return Cost<FUNCTION_TUKEY>(r2);
//  case FUNCTION_STUDENT_T_5:  return Cost<FUNCTION_STUDENT_T_5>(r2);
//  case FUNCTION_CAUCHY:   return Cost<FUNCTION_CAUCHY>(r2);
//  }
//  return 0.0f;
//}

template<int FUNCTION> inline int Count(const AlignedVector<float> &r2s) {
  int SN = 0;
  const float s2 = Variance<FUNCTION>();
  const int N = r2s.Size();
  for (int i = 0; i < N; ++i) {
    if (r2s[i] < s2) {
      ++SN;
    }
  }
  return SN;
}

template<int DATA_DIM, int PARAM_DOF> inline float Variance(AlignedVector<float> &r2s) {
  const int ith = r2s.Size() >> 1;
  std::nth_element(r2s.Data(), r2s.Data() + ith, r2s.End());
  const float t = 1.4826f * (1.0f + 5.0f / (r2s.Size() * DATA_DIM - PARAM_DOF));
  return t * t * r2s[ith];
}

template<int PARAM_DOF> inline float ChiSquareDistance(const float p, const float w = 1.0f);
template<> inline float ChiSquareDistance<3>(const float p, const float w) {
#ifdef CFG_DEBUG
  UT_ASSERT(p == 0.9f || p == 0.99f || p == 0.999f || p == 0.9999f || p == 0.99999f || p == 1.0f);
#endif
  float X2;
  if (p == 0.9f) {
    X2 = 0.5844f;
  } else if (p == 0.99f) {
    X2 = 0.1148f;
  } else if (p == 0.999f) {
    X2 = 0.0243f;
  } else if (p == 0.9999f) {
    X2 = 0.0052f;
  } else if (p == 0.99999f) {
    X2 = 0.0011f;
  }/* else if (p == 1.0f) {
    X2 = 0.0f;
  } else {
    X2 = 0.0f;
  } */else {
    return 0.0f;
  }
  return std::max(X2 * w, FLT_EPSILON);
}

template<> inline float Weight<FUNCTION_NONE>(const float r2) { return 1.0f; }
template<> inline void Weight<FUNCTION_NONE>(const AlignedVector<float> &r2s,
                                             AlignedVector<float> &ws) {
  LA::AlignedVectorXf _ws((float *) ws.Data(), ws.Size(), false);
  _ws.Set(1.0f);
}
template<> inline float Cost<FUNCTION_NONE>(const float r2) { return r2; }
template<> inline float Cost<FUNCTION_NONE>(const AlignedVector<float> &r2s) {
  LA::AlignedVectorXf _r2s((float *) r2s.Data(), r2s.Size(), false);
  return _r2s.Sum();
}

template<> inline float Weight<FUNCTION_HUBER>(const float r2) {
  if (r2 > ME_VARIANCE_HUBER) {
    return sqrtf(ME_VARIANCE_HUBER / r2);
  } else {
    return 1.0f;
  }
}
template<> inline void Weight<FUNCTION_HUBER>(const AlignedVector<float> &r2s,
                                              AlignedVector<float> &ws) {
  const int N = r2s.Size();
  ws.Resize(N);
  for (int i = 0; i < N; ++i) {
    ws[i] = Weight<FUNCTION_HUBER>(r2s[i]);
  }
}
template<> inline float Cost<FUNCTION_HUBER>(const float r2) {
  //if (r2 > ME_VARIANCE_HUBER) {
  //  return sqrtf(ME_VARIANCE_HUBER * r2) - ME_VARIANCE_HUBER * 0.5f;
  //} else {
  //  return r2 * 0.5f;
  //}
  if (r2 > ME_VARIANCE_HUBER) {
    return sqrtf(ME_VARIANCE_HUBER * r2) * 2.0f - ME_VARIANCE_HUBER;
  } else {
    return r2;
  }
}
template<> inline float Cost<FUNCTION_HUBER>(const AlignedVector<float> &r2s) {
  float C = 0.0f;
  const int N = r2s.Size();
  for (int i = 0; i < N; ++i) {
    C = Cost<FUNCTION_HUBER>(r2s[i]) + C;
  }
  return C;
}
template<> inline float Variance<FUNCTION_HUBER>() { return ME_VARIANCE_HUBER; }

template<> inline float Weight<FUNCTION_TUKEY>(const float r2) {
  if (r2 > ME_VARIANCE_TUKEY) {
    return 0.0f;
  }
  const float t = -(r2 * ME_VARIANCE_TUKEY_INVERSE) + 1.0f;
  return t * t;
}
template<> inline float Cost<FUNCTION_TUKEY>(const float r2) {
  //if (r2 > ME_VARIANCE_TUKEY) {
  //  return ME_VARIANCE_TUKEY / 6;
  //}
  //const float t = 1 - r2 / ME_VARIANCE_TUKEY;
  //return (1.0f - t * t * t) * ME_VARIANCE_TUKEY / 6;
  if (r2 > ME_VARIANCE_TUKEY) {
    return 1.0f;
  }
  const float t = -r2 * ME_VARIANCE_TUKEY_INVERSE + 1.0f;
  return -t * t * t + 1.0f;
}
template<> inline float Cost<FUNCTION_TUKEY>(const AlignedVector<float> &r2s) {
  // TODO (yanghongtian) : NEON implementation
  float C = 0.0f;
  const int N = r2s.Size();
  for (int i = 0; i < N; ++i) {
    C = Cost<FUNCTION_TUKEY>(r2s[i]) + C;
  }
  return C;
}
template<> inline float Variance<FUNCTION_TUKEY>() { return ME_VARIANCE_TUKEY; }

template<int DOF> inline float WeightStudentT(const float r2) {
  return DOF / (r2 * ME_VARIANCE_STUDENT_T_INVERSE + DOF);
}

template<int DOF> inline void WeightsStudentT(const AlignedVector<float> &r2s,
                                              AlignedVector<float> &ws) {
  // TODO (yanghongtian) : NEON implementation
  const int N = r2s.Size(), NF = SIMD_FLOAT_FLOOR(N);
  ws.Resize(N);
  for (int i = 0; i < N; ++i) {
    ws[i] = WeightStudentT<DOF>(r2s[i]);
  }
}
template<int DOF> inline float CostStudentT(const float r2) {
  return logf(r2 / (ME_VARIANCE_STUDENT_T * DOF) + 1.0f);
}
template<int DOF> inline float CostStudentT(const AlignedVector<float> &r2s) {
  float C = 0.0f;
  const xp128f one = xp128f::get(1.0f);
  const xp128f s2dI = xp128f::get(1.0f / (ME_VARIANCE_STUDENT_T * DOF));
  const xp128f *r2 = (xp128f *) r2s.Data();
  const int N = r2s.Size(), NF = SIMD_FLOAT_FLOOR(N);
  for (int i = 0; i < NF; i += 4, ++r2) {
    const xp128f t = *r2 * s2dI + one;
    C = logf(t[0]) + logf(t[1]) + logf(t[2]) + logf(t[3]) + C;
  }
  for (int i = NF; i < N; ++i) {
    C = logf(r2s[i] * s2dI[0] + 1.0f) + C;
  }
  return C;
}
template<> inline float Weight<FUNCTION_STUDENT_T_5>(const float r2) { return WeightStudentT<5>(r2); }
template<> inline float Weight<FUNCTION_CAUCHY>(const float r2) { return WeightStudentT<1>(r2); }
template<> inline void Weight<FUNCTION_STUDENT_T_5>(const AlignedVector<float> &r2s,
                                                    AlignedVector<float> &ws) { WeightsStudentT<5>(r2s, ws); }
template<> inline void Weight<FUNCTION_CAUCHY>(const AlignedVector<float> &r2s,
                                               AlignedVector<float> &ws) { WeightsStudentT<1>(r2s, ws); }
template<> inline float Cost<FUNCTION_STUDENT_T_5>(const float r2) { return CostStudentT<5>(r2); }
template<> inline float Cost<FUNCTION_CAUCHY>(const float r2) { return CostStudentT<1>(r2); }
template<> inline float Cost<FUNCTION_STUDENT_T_5>(const AlignedVector<float> &r2s) { return CostStudentT<5>(r2s); }
template<> inline float Cost<FUNCTION_CAUCHY>(const AlignedVector<float> &r2s) { return CostStudentT<1>(r2s); }

}

#endif
