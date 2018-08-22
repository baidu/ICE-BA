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
#ifndef _LINEAR_SYSTEM_H_
#define _LINEAR_SYSTEM_H_

namespace LA {
namespace LS {

template<typename TYPE> inline bool DecomposeLDL(const int N, TYPE **A, const TYPE *eps = NULL) {
  TYPE *at = A[N - 1];
  for (int i = 0; i < N; ++i) {
    TYPE *ai = A[i];
    const TYPE aii = ai[i];
    if ((eps && aii <= eps[i]) || (!eps && aii <= 0)) {
      memset(ai + i, 0, sizeof(TYPE) * (N - i));
      return false;
    }
    const TYPE mii = 1 / aii;
    ai[i] = mii;
    memcpy(at, ai + i + 1, sizeof(TYPE) * (N - i - 1));
    for (int k = i + 1; k < N; ++k) {
      ai[k] *= mii;
    }
    for (int j = i + 1; j < N; ++j) {
      const TYPE aij = at[j - i - 1];
      TYPE *aj = A[j];
      for (int k = j; k < N; ++k) {
        aj[k] -= aij * ai[k];
      }
    }
  }
  return true;
}

template<typename TYPE> inline bool MarginalizeLDL(const int N, TYPE **A, TYPE *b,
                                                   const int im, const int Nm,
                                                   TYPE *work, const TYPE *eps = NULL) {
  TYPE *at1 = work, *at2 = at1 + im;
  const int _im = im + Nm;
  bool scc = true;
  for (int i = im; i < _im; ++i) {
    TYPE *ai = A[i];
    const TYPE aii = ai[i];
    if ((eps && aii <= eps[i - im]) || (!eps && aii <= 0)) {
      memset(ai + i, 0, sizeof(TYPE) * (N - i));
      b[i] = 0;
      //return false;
      scc = false;
      continue;
    }
    const TYPE mii = 1 / aii;
    for (int j = 0; j < im; ++j) {
      at1[j] = A[j][i];
      A[j][i] *= mii;
    }
    //ai[i] = mii;
    ai[i] = 1;
    memcpy(at2, ai + i + 1, sizeof(TYPE) * (N - i - 1));
    for (int k = i + 1; k < N; ++k) {
      ai[k] *= mii;
    }
    b[i] *= mii;
    const TYPE bi = b[i];
    for (int j = 0; j < im; ++j) {
      const TYPE aij = at1[j];
      TYPE *aj = A[j];
      for (int k = j; k < im; ++k) {
        aj[k] -= aij * A[k][i];
      }
      for (int k = i + 1; k < N; ++k) {
        aj[k] -= aij * ai[k];
      }
      b[j] -= aij * bi;
    }
    for (int j = i + 1; j < N; ++j) {
      const TYPE aij = at2[j - i - 1];
      TYPE *aj = A[j];
      for (int k = j; k < N; ++k) {
        aj[k] -= aij * ai[k];
      }
      b[j] -= aij * bi;
    }
  }
  //return true;
  return scc;
}

template<typename TYPE> inline int RankLDL(const int N, TYPE **A, TYPE *work,
                                           const TYPE *eps/* = NULL*/) {
  //TYPE *at = A[N - 1];
  TYPE *at = work;
  for (int i = 0; i < N; ++i) {
    TYPE *ai = A[i];
    const TYPE aii = ai[i];
    if ((eps && aii <= eps[i]) || (!eps && aii <= 0)) {
      return i;
    }
    memcpy(at, ai + i + 1, sizeof(TYPE) * (N - i - 1));
    const TYPE mii = 1 / aii;
    ai[i] = mii;
    for (int k = i + 1; k < N; ++k) {
      ai[k] *= mii;
    }
    for (int j = i + 1; j < N; ++j) {
      TYPE *aj = A[j];
      const TYPE aij = at[j - i - 1];
      for (int k = j; k < N; ++k) {
        aj[k] = -aij * ai[k] + aj[k];
      }
    }
  }
  return N;
}

template<typename TYPE> inline bool SolveLDL(const int N, TYPE **A, TYPE *b,
                                             const TYPE *eps = NULL,
                                             const bool decomposed = false) {
  if (!decomposed && !DecomposeLDL<TYPE>(N, A, eps)) {
    return false;
  }
  for (int i = 0; i < N; ++i) {
    const TYPE *ai = A[i];
    const TYPE bi = b[i];
    for (int j = i + 1; j < N; ++j) {
      b[j] -= ai[j] * bi;
    }
    b[i] *= ai[i];
  }
  for (int i = N - 2; i >= 0; --i) {
    const TYPE *ai = A[i];
    TYPE bi = b[i];
    for (int j = i + 1; j < N; ++j) {
      bi -= ai[j] * b[j];
    }
    b[i] = bi;
  }
  return true;
}

template<typename TYPE> inline bool InverseLDL(const int N, TYPE **A, const TYPE *eps = NULL,
                                               const bool decomposed = false) {
  if (!decomposed && !DecomposeLDL<TYPE>(N, A, eps)) {
    return false;
  }
  for (int i = 0; i < N; ++i) {
    const TYPE *ai = A[i];
    for (int k = i + 1; k < N; ++k) {
      A[k][i] = -ai[k];
    }
    for (int j = i + 1; j < N; ++j) {
      const TYPE *aj = A[j];
      const TYPE bj = aj[i];
      for (int k = j + 1; k < N; ++k) {
        A[k][i] -= aj[k] * bj;
      }
      A[j][i] *= aj[j];
    }
    for (int j = N - 1; j > i; --j) {
      const TYPE *aj = A[j];
      TYPE bj = aj[i];
      for (int k = j + 1; k < N; ++k) {
        bj -= aj[k] * A[k][i];
      }
      A[j][i] = bj;
    }
    TYPE bi = ai[i];
    for (int k = i + 1; k < N; ++k) {
      bi -= ai[k] * A[k][i];
    }
    A[i][i] = bi;
  }
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      A[i][j] = A[j][i];
    }
  }
  return true;
}

}
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
template<int N, int Nm>
inline void EigenMarginalize(Eigen::Matrix<float, N, N> &e_A, Eigen::Matrix<float, N, 1> &e_b) {
  const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> e_M = e_A.block(Nm, 0, N - Nm, Nm) *
                                                                  (e_A.block(0, 0, Nm, Nm).inverse());
  e_A.block(Nm, 0, N - Nm, N) -= e_M * e_A.block(0, 0, Nm, N);
  e_b.block(Nm, 0, N - Nm, 1) -= e_M * e_b.block(0, 0, Nm, 1);
}
template<typename TYPE, int N>
inline Eigen::Matrix<TYPE, N, N> EigenSqrt(const Eigen::Matrix<TYPE, N, N> &e_A) {
  const Eigen::JacobiSVD<Eigen::Matrix<TYPE, N, N> > svd(e_A, Eigen::ComputeFullU);
  const Eigen::Matrix<TYPE, N, N> e_U = svd.matrixU();
  Eigen::Matrix<TYPE, N, 1> e_S = svd.singularValues();
  for (int i = 0; i < N; ++i) {
    e_S(i, 0) = sqrtf(e_S(i, 0));
  }
  return e_U * e_S.asDiagonal() * e_U.transpose();
}
template<typename TYPE>
inline void EigenDecomposeLDL(const Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                                  Eigen::RowMajor> &e_A,
                              Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                            Eigen::RowMajor> *e_L,
                              Eigen::Matrix<TYPE, Eigen::Dynamic, 1> *e_d, const TYPE *eps = NULL) {
  const int N = e_A.rows();
#ifdef CFG_DEBUG
  UT_ASSERT(e_A.cols() == N);
#endif
  std::vector<TYPE> data(N * N);
  std::vector<TYPE *> A(N);
  A[0] = data.data();
  for (int i = 1; i < N; ++i) {
    A[i] = A[i - 1] + N;
  }
  LA::LS::DecomposeLDL(N, A.data(), eps);
  e_L->resize(N, N);
  e_L->setIdentity();
  e_d->resize(N);
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      (*e_L)(j, i) = A[i][j];
    }
    const TYPE aii = A[i][i];
    (*e_d)(i, 0) = aii == 0 ? 0 : 1 / aii;
  }
}
template<typename TYPE>
inline TYPE EigenConditionNumber(const Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                                     Eigen::RowMajor> &e_A,
                                 Eigen::Matrix<TYPE, Eigen::Dynamic, 1> *e_s = NULL) {
  const Eigen::JacobiSVD<Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor> > e_svd(e_A);
  const int N = e_svd.singularValues().size();
  const TYPE cond = e_svd.singularValues()(0) / e_svd.singularValues()(N - 1);
  if (e_s) {
    *e_s = e_svd.singularValues();
  }
  return cond;
}
template<typename TYPE>
inline int EigenRankLU(const Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                           Eigen::RowMajor> &e_A) {
  const Eigen::FullPivLU<Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor> > e_lu(e_A);
  return int(e_lu.rank());
}
template<typename TYPE>
inline int EigenRankQR(const Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                           Eigen::RowMajor> &e_A) {
  const Eigen::ColPivHouseholderQR<Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic,
                                                 Eigen::RowMajor> > e_qr(e_A);
  return int(e_qr.rank());
}
#endif

#endif
