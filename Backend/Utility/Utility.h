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
#ifndef UTILITY_UTILITY_H_
#define UTILITY_UTILITY_H_

#include <stdint.h>

#include "SIMD.h"
#include <stdarg.h>  // for va_start, va_end
#include <list>
#include <string>
#include <algorithm>
#include <vector>

#if defined(_N) // for Android ndk
#undef _N
#undef _P
#undef _C
#endif

#define UT_STRING_WIDTH      79
#define UT_STRING_WIDTH_MAX  512
#define UT_FACTOR_RAD_TO_DEG 57.295779505601046646705075978956f
#define UT_FACTOR_DEG_TO_RAD 0.01745329252222222222222222222222f

#define UT_E          2.71828182845904523536f   // e
#define UT_LOG2E      1.44269504088896340736f   // log2(e)
#define UT_LOG10E     0.434294481903251827651f  // log10(e)
#define UT_LN2        0.693147180559945309417f  // ln(2)
#define UT_LN10       2.30258509299404568402f   // ln(10)
#define UT_PI         3.14159265358979323846f   // pi
#define UT_2PI        6.28318530717958647692f   // pi*2
#define UT_PI_2       1.57079632679489661923f   // pi/2
#define UT_PI_4       0.785398163397448309616f  // pi/4
#define UT_1_PI       0.318309886183790671538f  // 1/pi
#define UT_1_2PI      0.159154943091895335769f  // 1/(pi*2)
#define UT_2_PI       0.636619772367581343076f  // 2/pi
#define UT_2_SQRTPI   1.1283791670955125739f    // 2/sqrt(pi)
#define UT_SQRT2      1.4142135623730950488f    // sqrt(2)
#define UT_1_SQRT2    0.7071067811865475244f    // 1/sqrt(2)
#define UT_1_SQRT2PI  0.39894228040143267794f   // 1/sqrt(2pi)

#define UT_GET_STRING(format, str) {\
  if (format) {\
    va_list args;\
    va_start(args, format);\
    vsprintf(str, format, args);\
    va_end(args);\
  } else {\
    str[0] = 0;\
  }\
}\

#define UT_SWAP(a, b) {\
  const auto t = (a);\
  (a) = (b);\
  (b) = (t);\
}\

#define UT_CLAMP(v, vMin, vMax) std::min(std::max((v), (vMin)), (vMax))

#define UT_SINF(x) static_cast<float>(sin(static_cast<double>(x)))
#define UT_COSF(x) static_cast<float>(cos(static_cast<double>(x)))
#define UT_TANF(x) static_cast<float>(tan(static_cast<double>(x)))
#define UT_ASINF(x) static_cast<float>(asin(static_cast<double>(x)))
#define UT_ACOSF(x) static_cast<float>(acos(static_cast<double>(x)))
#define UT_ATANF(x) static_cast<float>(atan(static_cast<double>(x)))

#define UT_DOT_TO_ANGLE(d) UT_ACOSF(UT_CLAMP((d), -1.0f, 1.0f))

#define UT_ASSERT(expression) UT::Assert(expression, #expression)

#define UT_FLT_EPSILON_MIN        1.0e-05f
#define UT_FLT_EPSILON_MAX        1.0e-03f
#define UT_FLT_EPSILON_RATIO_MIN  1.0e-03f
#define UT_FLT_EPSILON_RATIO_MAX  0.01f

#define UT_DBL_EPSILON_MIN        1.0e-08
#define UT_DBL_EPSILON_MAX        1.0e-06
#define UT_DBL_EPSILON_RATIO_MIN  1.0e-05
#define UT_DBL_EPSILON_RATIO_MAX  0.001

namespace UT {
void Assert(const bool expression, const char *format, ...);
void DebugStart();
bool Debugging();
void DebugStop();
void PrintStart(const std::string fileName);
void PrintStop();
void Print(const char *format, ...);
void PrintSeparator(const char c = '-');
int PrintStringWidth();
void SaveSeparator(FILE *fp, const char c = '-');
inline void PrintLoaded(const char *fileName) {
#ifdef CFG_VERBOSE
  printf("Loaded \'%s\'\n", fileName);
#endif
}
inline void PrintSaved(const char *fileName) {
#ifdef CFG_VERBOSE
  printf("Saved \'%s\'\n", fileName);
#endif
}
inline void PrintLoaded(const std::string fileName) { PrintLoaded(fileName.c_str()); }
inline void PrintSaved(const std::string fileName) { PrintSaved(fileName.c_str()); }
template<typename TYPE> inline void PrintValue(const TYPE v, const bool e = false) {
  Print("%d", v);
}
template<> inline void PrintValue<float>(const float v, const bool e) {
  if (e) {
    Print("%e", v);
  } else {
    Print("%f", v);
  }
}
template<> inline void PrintValue<double>(const double v, const bool e) {
  if (e) {
    Print("%e", v);
  } else {
    Print("%f", v);
  }
}
template<typename TYPE>
inline void PrintError(const TYPE v1, const TYPE v2, const bool e = false) {
  Print("|");
  PrintValue<TYPE>(v1, e);
  Print(" - ");
  PrintValue<TYPE>(v2, e);
  Print("| = ");
  PrintValue<TYPE>(v1 >= v2 ? v1 - v2 : v2 - v1, e);
  Print("\n");
}

void Error(const char *format = NULL, ...);
void Check(const char *format = NULL, ...);

template<typename TYPE> inline TYPE Random();
template<> inline int Random<int>() {
  // Error("Random\n");
  return rand();
}  // [0, RAND_MAX]
template<> inline bool Random<bool>() { return (Random<int>() & 1) == 0; }
template<> inline ubyte Random<ubyte>() { return Random<int>() & 255; }
template<> inline ushort Random<ushort>() { return ushort(Random<int>()); }
template<> inline int16_t Random<int16_t>() { return int16_t(Random<int>() - 16384); }
template<> inline float Random<float>() { return static_cast<float>(Random<int>()) / RAND_MAX; }

template<typename TYPE> inline TYPE Random(const TYPE vMax);
template<> inline ubyte Random<ubyte>(const ubyte vMax) { return Random<int>() % vMax; }
template<> inline int Random<int>(const int vMax) { return Random<int>() % vMax; }
template<> inline float Random<float>(const float vMax) { return Random<float>() * vMax; }

template<typename TYPE> inline TYPE Random(const TYPE vMin, const TYPE vMax) {
  return TYPE(Random<float>() * (vMax - vMin) + vMin);
}

template<typename TYPE> inline void Random(TYPE *v, const int N) {
  for (int i = 0; i < N; ++i) {
    v[i] = Random<TYPE>();
  }
}
template<typename TYPE> inline void Random(TYPE *v, const int N, const TYPE vMax) {
  for (int i = 0; i < N; ++i) {
    v[i] = Random<TYPE>(vMax);
  }
}
template<typename TYPE> inline void Random(TYPE *v, const int N, const TYPE vMin,
                                           const TYPE vMax) {
  for (int i = 0; i < N; ++i) {
    v[i] = Random<TYPE>(vMin, vMax);
  }
}

inline int Random(std::vector<ubyte> &marks) {
  const int N = static_cast<int>(marks.size());
  while (1) {
    const int i = Random<int>(N);
    if (marks[i]) {
      continue;
    }
    marks[i] = 1;
    return i;
  }
}

inline float RandomUniform() {
  const float x = Random<float>();
  return x + x - 1.0f;
}
inline float RandomGaussian() {
  float r, t;
  while ((r = Random<float>()) == 0.0f) {}
  t = Random<float>() * UT_2PI;
  return sqrtf(-2.0f * logf(r)) * UT_COSF(t);
}
inline void RandomGaussian(float &x, float &y) {
  float w;
  while (1) {
    x = RandomUniform();
    y = RandomUniform();
    if ((w = x * x + y * y) < 1.0f) {
      break;
    }
  }
  w = sqrtf((-2.0f * logf(w)) / w);
  x *= w;
  y *= w;
}

template<typename TYPE> inline TYPE Epsilon();
template<> inline float Epsilon<float>() { return FLT_EPSILON; }
template<> inline double Epsilon<double>() { return DBL_EPSILON; }
template<typename TYPE> inline TYPE EpsilonMin();
template<> inline float EpsilonMin<float>() { return UT_FLT_EPSILON_MIN; }
template<> inline double EpsilonMin<double>() { return UT_DBL_EPSILON_MIN; }
template<typename TYPE> inline TYPE EpsilonMax();
template<> inline float EpsilonMax<float>() { return UT_FLT_EPSILON_MAX; }
template<> inline double EpsilonMax<double>() { return UT_DBL_EPSILON_MAX; }
template<typename TYPE> inline TYPE EpsilonRatioMin();
template<> inline float EpsilonRatioMin<float>() { return UT_FLT_EPSILON_RATIO_MIN; }
template<> inline double EpsilonRatioMin<double>() { return UT_DBL_EPSILON_RATIO_MIN; }
template<typename TYPE> inline TYPE EpsilonRatioMax();
template<> inline float EpsilonRatioMax<float>() { return UT_FLT_EPSILON_RATIO_MAX; }
template<> inline double EpsilonRatioMax<double>() { return UT_DBL_EPSILON_RATIO_MAX; }
template<typename TYPE> inline TYPE Invalid();
template<> inline ubyte Invalid<ubyte>() { return UCHAR_MAX; }
template<> inline int16_t Invalid<int16_t>() { return SHRT_MAX; }
template<> inline ushort Invalid<ushort>() { return USHRT_MAX; }
template<> inline int Invalid<int>() { return INT_MAX; }
template<> inline float Invalid<float>() { return FLT_MAX; }
template<> inline double Invalid<double>() { return DBL_MAX; }
template<typename TYPE> inline bool IsNAN(const TYPE v);
#ifdef WIN32
template<> inline bool IsNAN<float>(const float v) { return _isnanf(v) != 0; }
template<> inline bool IsNAN<double>(const double v) { return _isnan(v) != 0; }
#else
template<> inline bool IsNAN<float>(const float v) { return std::isnan(v) != 0; }
template<> inline bool IsNAN<double>(const double v) { return std::isnan(v) != 0; }
#endif
template<typename TYPE> inline TYPE Inverse(const TYPE v, const TYPE s = 1, const TYPE eps = 0) {
  return v == 0 ? 0 : std::max(s / v, eps);
}
template<typename TYPE> inline float Percentage(const TYPE vn, const TYPE vd) {
  return vd == 0 ? 0.0f : static_cast<float>(vn * 100) / vd;
}

template<typename TYPE> inline void Save(const TYPE v, FILE *fp);
template<> inline void Save<int>(const int v, FILE *fp) { fprintf(fp, "%d\n", v); }
template<> inline void Save<float>(const float v, FILE *fp) { fprintf(fp, "%f\n", v); }
template<> inline void Save<double>(const double v, FILE *fp) { fprintf(fp, "%lf\n", v); }
template<typename TYPE> inline bool Load(TYPE &v, FILE *fp);
template<> inline bool Load<int>(int &v, FILE *fp) { return fscanf(fp, "%d", &v) == 1; }
template<> inline bool Load<float>(float &v, FILE *fp) { return fscanf(fp, "%f", &v) == 1; }
template<> inline bool Load<double>(double &v, FILE *fp) { return fscanf(fp, "%lf", &v) == 1; }
template<typename TYPE> inline TYPE Load(FILE *fp) {
  TYPE v;
  if (Load<TYPE>(v, fp)) {
    return v;
  } else {
    return Invalid<TYPE>();
  }
}
template<typename TYPE> inline bool Load(TYPE *V, const int N, FILE *fp) {
  for (int i = 0; i < N; ++i) {
    if (!Load<TYPE>(V[i], fp)) {
      return false;
    }
  }
  return true;
}
template<class TYPE> inline void SaveB(const TYPE &v, FILE *fp) {
  fwrite(&v, sizeof(TYPE), 1, fp);
}
template<class TYPE> inline void SaveB(const TYPE *v, const int N, FILE *fp) {
  if (N > 0) {
    fwrite(v, sizeof(TYPE), N, fp);
  }
}
template<class TYPE> inline void LoadB(TYPE &v, FILE *fp) { fread(&v, sizeof(TYPE), 1, fp); }
template<class TYPE> inline void LoadB(TYPE *v, const int N, FILE *fp) {
  if (N > 0) {
    fread(v, sizeof(TYPE), N, fp);
  }
}
template<typename TYPE> inline TYPE LoadB(FILE *fp) { TYPE v; LoadB<TYPE>(v, fp); return v; }
template<class TYPE> inline float MemoryMB(const int N = 1) { return sizeof(TYPE) * N / 1024.0f; }

bool FileExists(const std::string fileName);
FILE* FileOpen(const std::string fileName, const char *mode, const char *format, ...);
bool FileCopy(const std::string fileNameSrc, const std::string fileNameDst);
bool FileDelete(const std::string fileName, const bool check = true, const bool verbose = true);
std::vector<std::string> FilesSearch(const std::string fileNameFirst,
                                     const int iStart = 0,
                                     const int iStep = 1,
                                     const int iEnd = INT_MAX,
                                     const bool verbose = true);
std::vector<std::string> FilesSearch(const std::vector<std::string> &fileNames,
                                     const std::string dir,
                                     const std::string ext);
void FilesStartSaving(const std::string fileNameFirst,
                      const bool check = true,
                      const bool verbose = true);

std::string FileNameExtractDirectory(const std::string fileName);
std::string FileNameRemoveDirectory(const std::string fileName);
std::string FileNameExtractExtension(const std::string fileName);
std::string FileNameRemoveExtension(const std::string fileName);
std::string FileNameRemoveDirectoryExtension(const std::string fileName);
std::string FileNameRemovePrefix(const std::string fileName, const std::string prefix);
std::string FileNameAppendSuffix(const std::string fileName, const std::string suffix);
std::string FileNameAppendSuffix(const std::string fileName, const int suffix);
std::string FileNameAppendSuffix(const std::string fileName);
std::string FileNameReplaceDirectory(const std::string fileName, const std::string dirSrc,
                                     const std::string dirDst);
template<typename TYPE> inline TYPE FileNameExtractSuffix(const std::string fileName);
template<> inline int FileNameExtractSuffix<int>(const std::string fileName) {
  if (fileName == "") {
    return -1;
  }
  int i2 = static_cast<int>(fileName.length() - 1);
  if (!isdigit(fileName[i2])) {
    ++i2;
    while (--i2 >=0 && fileName[i2] != '.') {}
    if (--i2 < 0 || !isdigit(fileName[i2])) {
      return -1;
    }
  }
  int i1 = ++i2;
  while (--i1 >= 0 && isdigit(fileName[i1])) {}
  if (++i1 == i2) {
    return -1;
  } else {
    return atoi(fileName.substr(i1, i2 - i1).c_str());
  }
}
template<> inline double FileNameExtractSuffix<double>(const std::string fileName) {
  const std::string _fileName = FileNameRemoveDirectoryExtension(fileName).c_str();
  const int N = static_cast<int>(_fileName.length());
  for (int i = 0; i < N; ++i) {
    if (isdigit(_fileName[i])) {
      return atof(_fileName.substr(i, N - i).c_str());
    }
  }
  return DBL_MAX;
}
std::string FileNameIncreaseSuffix(const std::string fileName, const int incr);

std::string String(const char *format, ...);
std::string StringInput();
std::string StringReplace(const std::string str, const std::string strSrc,
                          const std::string strDst);
void StringSaveB(const std::string &str, FILE *fp);
void StringLoadB(std::string &str, FILE *fp);
void StringsSaveB(const std::vector<std::string> &strs, FILE *fp);
void StringsLoadB(std::vector<std::string> &strs, FILE *fp);
inline float StringMemoryMB(const std::string &str) {
  return MemoryMB<char>(static_cast<int>(str.size()));
}

std::vector<std::string> Strings(const std::string str0);
std::vector<std::string> Strings(const std::string str0, const std::string str1);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4, const std::string str5);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4, const std::string str5,
                                 const std::string str6);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4, const std::string str5,
                                 const std::string str6, const std::string str7);
std::vector<std::string> Strings(const std::string *strs, const int N);

template<class TYPE_SRC, class TYPE_DST>
inline std::vector<const TYPE_DST *> Pointers(const std::vector<TYPE_SRC> &V) {
  std::vector<const TYPE_DST *> Vptr;
  const int N = static_cast<int>(V.size());
  Vptr.resize(N);
  for (int i = 0; i < N; ++i) {
    Vptr[i] = &V[i];
  }
  return Vptr;
}
template<class TYPE_SRC_1, class TYPE_SRC_2, class TYPE_DST>
inline std::vector<const TYPE_DST *> Pointers(const std::vector<TYPE_SRC_1> &V1,
                                              const std::vector<TYPE_SRC_2> &V2) {
  std::vector<const TYPE_DST *> Vptr;
  const int N1 = static_cast<int>(V1.size());
  for (int i = 0; i < N1; ++i) {
    Vptr.push_back(&V1[i]);
  }
  const int N2 = static_cast<int>(V2.size());
  for (int i = 0; i < N2; ++i) {
    Vptr.push_back(&V2[i]);
  }
  return Vptr;
}
void SaveValues(const std::string fileName, const std::vector<float> &vs);
void SaveHistogram(const std::string fileName, const std::vector<float> &vs, const int Nb,
                   const float vMin = FLT_MAX, const float vMax = FLT_MAX);

template<typename TYPE>
inline bool Equal(const TYPE v1, const TYPE v2, const TYPE epsAbs = 0, const TYPE epsRel = 0) {
  if ((epsAbs < 0 || epsRel < 0) && v1 != v2) {
    return false;
  }
  const TYPE e = fabs(v1 - v2), va1 = fabs(v1), va2 = fabs(v2);
  const TYPE vMin = std::min(va1, va2), vMax = std::max(va1, va2);
  const TYPE vd = vMin >= EpsilonMin<TYPE>() ? vMin : vMax, er = e / vd;
  return e <= epsAbs || er <= epsRel ||
         e <= EpsilonMin<TYPE>() || er <= EpsilonRatioMin<TYPE>() ||
         (e <= EpsilonMax<TYPE>() && er <= EpsilonRatioMax<TYPE>());
}
template<typename TYPE>
inline bool AssertEqual(const TYPE v1, const TYPE v2,
                        const int verbose = 1, const std::string str = "",
                        const TYPE epsAbs = 0, const TYPE epsRel = 0) {
  if (Equal<TYPE>(v1, v2, epsAbs, epsRel)) {
    return true;
  }
  // EXPECT_EQ(v1, v2);
  if (verbose > 0) {
    PrintSeparator();
    if (str != "") {
      Print("%s\n", str.c_str());
    }
    PrintError<TYPE>(v1, v2, verbose > 1);
  }
  return Equal(v1, v2, epsAbs, epsRel);
}
template<typename TYPE>
inline bool AssertZero(const TYPE v, const int verbose = 1, const std::string str = "",
                       const TYPE epsAbs = 0, const TYPE epsRel = 0) {
  return AssertEqual<TYPE>(0, v, verbose, str, epsAbs, epsRel);
}

template<class TYPE> inline void VectorMakeZero(std::vector<TYPE> &V) {
  memset(V.data(), 0, sizeof(TYPE) * V.size());
}
template<class TYPE> inline
void VectorMakeZero(std::vector<TYPE> &V, const int i1, const int i2) {
  memset(V.data() + i1, 0, sizeof(TYPE) * (i2 - i1));
}
template<class TYPE> inline int VectorCount(const std::vector<TYPE> &V, const TYPE &v) {
  int SN = 0;
  const int N = static_cast<int>(V.size());
  for (int i = 0; i < N; ++i) {
    if (V[i] == v) {
      ++SN;
    }
  }
  return SN;
}
template<class TYPE> inline int VectorCountNonZero(const std::vector<TYPE> &V) {
  int SN = 0;
  const int N = static_cast<int>(V.size());
  for (int i = 0; i < N; ++i) {
    if (V[i]) {
      ++SN;
    }
  }
  return SN;
}
template<class TYPE> inline int VectorCountFlag(const std::vector<TYPE> &V, const TYPE flag) {
  int SN = 0;
  const int N = static_cast<int>(V.size());
  for (int i = 0; i < N; ++i) {
    if (V[i] & flag) {
      ++SN;
    }
  }
  return SN;
}
template<class TYPE> inline int VectorSearchFirstFlagNot(const std::vector<TYPE> &V, const int i1,
                                                         const int i2, const TYPE flag) {
  for (int i = i1; i < i2; ++i) {
    if (!(V[i] & flag)) {
      return i;
    }
  }
  return -1;
}
template<class TYPE> inline bool VectorExistFlag(const TYPE *V, const int N, const TYPE flag) {
  for (int i = 0; i < N; ++i) {
    if (V[i] & flag) {
      return true;
    }
  }
  return false;
}
template<class TYPE> inline bool VectorExistFlagNot(const TYPE *V, const int N, const TYPE flag) {
  for (int i = 0; i < N; ++i) {
    if (!(V[i] & flag)) {
      return true;
    }
  }
  return false;
}
template<class TYPE> inline TYPE VectorLength(const TYPE *V, const int N) {
  double Sv = 0.0;
  for (int i = 0; i < N; ++i) {
    Sv = static_cast<double>(V[i]) * V[i] + Sv;
  }
  return TYPE(sqrt(Sv));
}
template<class TYPE_1, class TYPE_2> inline bool VectorEqual(const TYPE_1 *V1, const TYPE_2 *V2,
                                                             const int N) {
  bool equal = true;
  for (int i = 0; i < N && equal; ++i) {
    equal = V1[i] == V2[i];
  }
  return equal;
}
template<class TYPE_1, class TYPE_2> inline bool VectorEqual(const std::vector<TYPE_1> &V1,
                                                             const std::vector<TYPE_2> &V2) {
  return V1.size() == V2.size() &&
    VectorEqual<TYPE_1, TYPE_2>(V1.data(), V2.data(), static_cast<int>(V1.size()));
}
template<typename TYPE>
inline bool VectorAssertEqual(const TYPE *V1, const TYPE *V2, const int N,
                              const int verbose = 1, const std::string str = "",
                              const TYPE epsAbs = 0, const TYPE epsRel = 0) {
  bool equal = true;
  const TYPE v1 = VectorLength(V1, N), v2 = VectorLength(V2, N);
  const TYPE vMin = std::min(v1, v2), vMax = std::max(v1, v2);
  const TYPE vd = vMin >= EpsilonMin<TYPE>() ? vMin : vMax;
  const TYPE _epsRel = epsRel < 0 ? epsRel : std::max(epsRel, EpsilonRatioMin<TYPE>());
  const TYPE _epsAbs = epsAbs < 0 ? epsAbs : std::max(epsAbs, vd * _epsRel);
  const int verbose1 = verbose & 3, verbose2 = verbose >> 2;
  for (int i = 0; i < N && equal; ++i) {
    equal = AssertEqual<TYPE>(V1[i], V2[i], verbose1, str, _epsAbs);
  }
  if (!equal && verbose2 > 0) {
    const bool e = verbose2 > 1;
    PrintSeparator();
    for (int i = 0; i < N; ++i) {
      PrintError<TYPE>(V1[i], V2[i], e);
    }
  }
  return equal;
}
template<class TYPE_1, class TYPE_2> inline void VectorAssertEqual(const std::vector<TYPE_1> &V1,
                                                                   const std::vector<TYPE_2> &V2) {
  UT_ASSERT(VectorEqual(V1, V2));
}
template<typename TYPE>
inline bool VectorAssertZero(const TYPE *V, const int N,
                             const int verbose = 1, const std::string str = "",
                             const float epsAbs = 0.0f, const float epsRel = 0.0f) {
  bool zero = true;
  const TYPE v = VectorLength(V, N);
  const TYPE _epsRel = epsRel < 0 ? epsRel : std::max(epsRel, EpsilonRatioMin<TYPE>());
  const TYPE _epsAbs = epsAbs < 0 ? epsAbs : std::max(epsAbs, v * _epsRel);
  const int verbose1 = verbose & 3, verbose2 = verbose >> 2;
  for (int i = 0; i < N && zero; ++i) {
    zero = AssertEqual<TYPE>(0, V[i], verbose1, str, epsAbs, epsRel);
  }
  if (!zero && verbose2 > 0) {
    const bool e = verbose2 > 1;
    PrintSeparator();
    for (int i = 0; i < N; ++i) {
      PrintError<TYPE>(V[i], 0, e);
    }
  }
  return zero;
}
template<class TYPE> inline std::vector<TYPE> VectorRepeat(const std::vector<TYPE> &V,
                                                           const int N) {
  if (N <= 0) {
    return std::vector<TYPE>();
  } else if (N == 1) {
    return V;
  }
  std::vector<TYPE> _V;
  for (int i = 0; i < N; ++i) {
    _V.insert(_V.end(), V.begin(), V.end());
  }
  return _V;
}
template<class TYPE> inline void VectorSave(const std::vector<TYPE> &V, FILE *fp) {
  const int N = static_cast<int>(V.size());
  Save<int>(N, fp);
  for (int i = 0; i < N; ++i) {
    V[i].Save(fp);
  }
}
template<class TYPE> inline void VectorLoad(std::vector<TYPE> &V, FILE *fp) {
  const int N = Load<int>(fp);
  V.resize(N);
  for (int i = 0; i < N; ++i) {
    V[i].Load(fp);
  }
}
template<class TYPE> inline void VectorSaveB(const SIMD::vector<TYPE> &V, FILE *fp) {
  const int N = static_cast<int>(V.size());
  SaveB<int>(N, fp);
  SaveB<TYPE>(V.data(), N, fp);
}
template<class TYPE> inline void VectorSaveB(const std::vector<TYPE> &V, FILE *fp) {
  const int N = static_cast<int>(V.size());
  SaveB<int>(N, fp);
  SaveB<TYPE>(V.data(), N, fp);
}
template<class TYPE> inline bool VectorSaveB(const std::vector<TYPE> &V, const char *fileName) {
  FILE *fp = fopen(fileName, "wb");
  if (!fp) {
    return false;
  }
  VectorSaveB<TYPE>(V, fp);
  fclose(fp);
  PrintSaved(fileName);
  return true;
}
template<class TYPE> inline int VectorLoadB(SIMD::vector<TYPE> &V, FILE *fp) {
  const int N = LoadB<int>(fp);
  V.resize(N);
  LoadB<TYPE>(V.data(), N, fp);
  return N;
}
template<class TYPE> inline int VectorLoadB(std::vector<TYPE> &V, FILE *fp) {
  const int N = LoadB<int>(fp);
  V.resize(N);
  LoadB<TYPE>(V.data(), N, fp);
  return N;
}
template<class TYPE> inline bool VectorLoadB(std::vector<TYPE> &V, const char *fileName) {
  FILE *fp = fopen(fileName, "rb");
  if (!fp) {
    return false;
  }
  VectorLoadB<TYPE>(V, fp);
  fclose(fp);
  PrintLoaded(fileName);
  return true;
}
template<class TYPE> inline void VectorsSaveB(const std::vector<std::vector<TYPE> > &Vs, FILE *fp) {
  const int N = static_cast<int>(Vs.size());
  SaveB<int>(N, fp);
  for (int i = 0; i < N; ++i) {
    VectorSaveB(Vs[i], fp);
  }
}
template<class TYPE> inline void VectorsLoadB(std::vector<std::vector<TYPE> > &Vs, FILE *fp) {
  const int N = LoadB<int>(fp);
  Vs.resize(N);
  for (int i = 0; i < N; ++i) {
    VectorLoadB(Vs[i], fp);
  }
}
template<class TYPE> inline void VectorsSaveB(const std::list<std::vector<TYPE> > &Vs, FILE *fp) {
  const int N = static_cast<int>(Vs.size());
  SaveB<int>(N, fp);
  for (typename std::list<std::vector<TYPE> >::const_iterator i = Vs.begin(); i != Vs.end(); ++i) {
    VectorSaveB<TYPE>(*i, fp);
  }
}
template<class TYPE> inline void VectorsLoadB(std::list<std::vector<TYPE> > &Vs, FILE *fp) {
  const int N = LoadB<int>(fp);
  Vs.resize(N);
  for (typename  std::list<std::vector<TYPE> >::iterator i = Vs.begin(); i != Vs.end(); ++i) {
    VectorLoadB<TYPE>(*i, fp);
  }
}
template<class TYPE> inline float VectorMemoryMB(const std::vector<TYPE> &V) {
  return MemoryMB<TYPE>(static_cast<int>(V.capacity()));
}
template<class TYPE> inline float VectorsMemoryMB(const std::vector<std::vector<TYPE> > &Vs) {
  float sum = 0.0f;
  const int N = static_cast<int>(Vs.size());
  for (int i = 0; i < N; ++i) {
    sum = VectorMemoryMB(Vs[i]) + sum;
  }
  return sum;
}

template<class TYPE> inline void ListSaveB(const std::list<TYPE> &L, FILE *fp) {
  const int N = static_cast<int>(L.size());
  SaveB<int>(N, fp);
  for (typename std::list<TYPE>::const_iterator i = L.begin(); i != L.end(); ++i) {
    SaveB<TYPE>(*i, fp);
  }
}
template<class TYPE> inline void ListLoadB(std::list<TYPE> &L, FILE *fp) {
  const int N = LoadB<int>(fp);
  L.resize(N);
  for (typename std::list<TYPE>::iterator i = L.begin(); i != L.end(); ++i) {
    LoadB<TYPE>(*i, fp);
  }
}

inline void ImageInterpolateWeight(const float dx, const float dy, xp128f &w) {
  w[3] = dx * dy;
  w[2] = dx - w[3];
  w[1] = dy - w[3];
  w[0] = 1.0f - dy - w[2];
}
template<typename TYPE_SRC, typename TYPE_DST>
inline void ImageInterpolate(const TYPE_SRC i11, const TYPE_SRC i12, const TYPE_SRC i21,
                             const TYPE_SRC i22, const xp128f &w, TYPE_DST &ip);
template<> inline void ImageInterpolate<ubyte, ubyte>(const ubyte i11, const ubyte i12,
                                                      const ubyte i21, const ubyte i22,
                                                      const xp128f &w, ubyte &ip) {
  xp128f tmp;
  tmp.vset_all_lane(static_cast<float>(i11), static_cast<float>(i12),
                     static_cast<float>(i21), static_cast<float>(i22));
  ip = static_cast<ubyte>((tmp * w).vsum_all() + 0.5f);
}
template<> inline void ImageInterpolate<ushort, ushort>(const ushort i11, const ushort i12,
                                                        const ushort i21, const ushort i22,
                                                        const xp128f &w, ushort &ip) {
  xp128f tmp;
  tmp.vset_all_lane(static_cast<float>(i11), static_cast<float>(i12),
                     static_cast<float>(i21), static_cast<float>(i22));
  ip = static_cast<ushort>((tmp * w).vsum_all() + 0.5f);
}
template<> inline void ImageInterpolate<ubyte, float>(const ubyte i11, const ubyte i12,
                                                      const ubyte i21, const ubyte i22,
                                                      const xp128f &w, float &ip) {
  xp128f tmp;
  tmp.vset_all_lane(static_cast<float>(i11), static_cast<float>(i12),
                     static_cast<float>(i21), static_cast<float>(i22));
  ip = (tmp * w).vsum_all();
}
template<> inline void ImageInterpolate<int16_t, float>(const int16_t i11, const int16_t i12,
                                                      const int16_t i21, const int16_t i22,
                                                      const xp128f &w, float &ip) {
  xp128f tmp;
  tmp.vset_all_lane(static_cast<float>(i11), static_cast<float>(i12),
                     static_cast<float>(i21), static_cast<float>(i22));
  ip = (tmp * w).vsum_all();
}
template<> inline void ImageInterpolate<ushort, float>(const ushort i11, const ushort i12,
                                                       const ushort i21, const ushort i22,
                                                       const xp128f &w, float &ip) {
  xp128f tmp;
  tmp.vset_all_lane(static_cast<float>(i11), static_cast<float>(i12),
                     static_cast<float>(i21), static_cast<float>(i22));
  ip = (tmp * w).vsum_all();
}
template<> inline void ImageInterpolate<float, float>(const float i11, const float i12,
                                                      const float i21, const float i22,
                                                      const xp128f &w, float &ip) {
  xp128f tmp; tmp.vset_all_lane(i11, i12, i21, i22);
  ip = (tmp * w).vsum_all();
}

template<typename TYPE> inline TYPE Input();
template<> inline std::string Input<std::string>() { return StringInput(); }
template<> inline int Input<int>() {
  const std::string input = Input<std::string>();
  int _input;
  sscanf(input.c_str(), "%d", &_input);
  return _input;
}
template<> inline char Input<char>() {
  const std::string input = Input<std::string>();
  char _input;
  sscanf(input.c_str(), "%c", &_input);
  return _input;
}
template<typename TYPE> inline TYPE Input(const char *format, ...) {
  char str[UT_STRING_WIDTH_MAX];
  UT_GET_STRING(format, str);
  printf("%s << ", str);
  return Input<TYPE>();
}

template<class ERROR> inline float ESSquaredLength(const ERROR &e) { return e.SquaredLength(); }
template<> inline float ESSquaredLength<float>(const float &e) { return e * e; }
template<class ERROR> inline void ESErrorPrint(const ERROR &e, const bool l = true) { e.Print(l); }
template<> inline void ESErrorPrint<float>(const float &e, const bool l) {
  if (l) {
    Print("%f", e);
  } else {
    Print("%.2f", e);
  }
}
template<class INDEX> inline void ESIndexPrint(const INDEX &idx) { idx.Print(); }
template<> inline void ESIndexPrint<int>(const int &idx) {
  if (idx != -1) {
    Print(" [%d]", idx);
  }
}
template<class ERROR, class INDEX> class ES {
 public:
  inline void Initialize(const bool r = false) {
    m_Se2 = m_SF = 0.0f;
    m_SN = 0;
    m_SNr = r ? 0 : -1;
    m_e2Max = m_FMax = -FLT_MAX;
  }
  inline void Accumulate(const ERROR &e, const float F = -1.0f, const INDEX idx = -1,
                         const bool r = true) {
    const float e2 = ESSquaredLength<ERROR>(e);
    if (e2 == -1.0f) {
      m_Se2 = -1.0f;
    } else {
      m_Se2 = e2 + m_Se2;
    }
    if (F == -1.0f) {
      m_SF = -1.0f;
    } else {
      m_SF = F + m_SF;
    }
    ++m_SN;
    if (m_SNr != -1 && r) {
      ++m_SNr;
    }
    if ((F == -1.0f && e2 > m_e2Max) ||
        (F != -1.0f && (F > m_FMax || (F == m_FMax && e2 > m_e2Max)))) {
      m_e2Max = e2;
      m_FMax = F;
      m_eMax = e;
      m_idxMax = idx;
    }
  }
  inline bool Valid() const { return m_SN > 0; }
  inline float Mean() const {
    return m_SN == 0 ? 0.0f : sqrtf(m_Se2 / m_SN);
  }
  inline float Maximal() const {
    return m_e2Max == -FLT_MAX ? 0.0f : sqrtf(m_e2Max);
  }
  inline float Total() const { return m_SF; }
  inline void Print(const std::string str = "", const bool F = true, const bool l = true,
                    const bool i = true, const int r = 0, const bool n = true) const {
    UT::Print("%s", str.c_str());
    if (F && m_SF != -1.0f) {
      if (l) {
        UT::Print("%e", m_SF);
      } else {
        UT::Print("%.2e", m_SF);
      }
    }
    if (m_Se2 != -1.0f) {
      if (F && m_SF != -1.0f) {
        UT::Print(" <-- ");
      }
      const float e = Mean();
      if (l) {
        UT::Print("%f", e);
      } else {
        UT::Print("%.2f", e);
      }
    }
    UT::Print(" <= ");
    ESErrorPrint(m_eMax, l);
    if (F && m_FMax != -FLT_MAX) {
      UT::Print(" --> ");
      if (l) {
        UT::Print("%e", m_FMax);
      } else {
        UT::Print("%.2e", m_FMax);
      }
    }
    if (i) {
      ESIndexPrint<INDEX>(m_idxMax);
    }
    if (r >= 1) {
      UT::Print(" (");
      if (r >= 2) {
        UT::Print("%d / %d = ", m_SNr, m_SN);
      }
      UT::Print("%d%%)", static_cast<int>(UT::Percentage(m_SNr, m_SN) + 0.5f));
    }
    if (n) {
      UT::Print("\n");
    }
  }

 public:
  float m_Se2, m_SF;
  int m_SN, m_SNr;
  float m_e2Max, m_FMax;
  ERROR m_eMax;
  INDEX m_idxMax;
};

template<class TYPE> inline
bool AssertReduction(const TYPE &v1, const TYPE &v2,
                     const int verbose = 1, const std::string str = "",
                     const float vMin = FLT_EPSILON, const float vMax = FLT_MAX,
                     const float eps = FLT_EPSILON) {
  const float v1_2 = v1.SquaredLength();
  const float v2_2 = v2.SquaredLength();
  if (v2_2 <= v1_2 + eps ||  std::max(v1_2, v2_2) < vMin * vMin) {
    return true;
  } else if (v1_2 > vMax * vMax) {
    return false;
  }
  // EXPECT_GE(v1_2, v2_2);
  if (verbose > 0) {
    PrintSeparator();
    if (str != "") {
      Print("%s\n", str.c_str());
    }
    if (verbose > 1) {
      Print("    %e: ", v1_2);  v1.Print(true);
      Print("--> %e: ", v2_2);  v2.Print(true);
    } else {
      Print("    %f: ", v1_2);  v1.Print(false);
      Print("--> %f: ", v2_2);  v2.Print(false);
    }
  }
  return false;
}
template<> inline
bool AssertReduction<float>(const float &v1, const float &v2, const int verbose,
                            const std::string str, const float vMin, const float vMax,
                            const float eps) {
  const float v1_a = fabs(v1);
  const float v2_a = fabs(v2);
  if (v2_a <= v1_a + eps || std::max(v1_a, v2_a) < vMin) {
    return true;
  } else if (v1_a > vMax) {
    return false;
  }
  // EXPECT_GE(v1_a, v2_a);
  if (verbose > 0) {
    PrintSeparator();
    if (str != "") {
      Print("%s\n", str.c_str());
    }
    if (verbose > 1) {
      Print("    %e --> %e\n", v1, v2);
    } else {
      Print("    %f --> %f\n", v1, v2);
    }
  }
  return false;
}
inline void PrintReduction(const float v1, const float v2,
                           const std::string str = "", const bool e = false,
                           const bool n = true) {
  if (str != "") {
    Print("%s", str.c_str());
  }
  if (e) {
    Print("%e --> %e", v1, v2);
  } else {
    Print("%f --> %f", v1, v2);
  }
  if (v1 < v2) {
    Print("*");
  }
  if (n) {
    Print("\n");
  }
}
inline void PrintReduction(const float v1, const float *v2, const int N2,
                           const std::string str = "", const bool e = false,
                           const bool n = true) {
  Print("%s", str.c_str());
  if (e) {
    Print("%e", v1);
    for (int i = 0; i < N2; ++i) {
      if (v2[i] != FLT_MAX) {
        Print(" --> %e", v2[i]);
      }
    }
  } else {
    Print("%f", v1);
    for (int i = 0; i < N2; ++i) {
      if (v2[i] != FLT_MAX) {
        Print(" --> %f", v2[i]);
      }
    }
  }
  for (int i = 0; i < N2; ++i) {
    if ((v2[i] != FLT_MAX && v1 < v2[i])) {
      Print("*");
      break;
    }
  }
  if (n) {
    Print("\n");
  }
}
}  // namespace UT

#endif  // UTILITY_UTILITY_H_
